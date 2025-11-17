// src/addr_mapper/impl/hbm4_group_mapper.cpp
#include <vector>
#include <algorithm>          // std::min
#include <cstdint>            // uint64_t
#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"
#include "spdlog/spdlog.h"

namespace Ramulator {

// HBM4 스타일의 "채널 그룹(CS 게이팅) 인지" 매퍼
class HBM4_GroupMapper final : public IAddrMapper, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IAddrMapper, HBM4_GroupMapper,
      "HBM4_GroupMapper",
      "Channel-group–aware mapping (HBM4-style CS gating).");

  // --- 내부 상태 ---
  IDRAM* m_dram = nullptr;

  int m_num_levels = -1;
  std::vector<int> m_addr_bits;
  Addr_t m_tx_offset = -1;

  int m_row_bits_idx = -1;
  int m_col_bits_idx = -1;

  int m_total_channels = -1;

  // 정책 파라미터(필요시 YAML로 노출)
  int m_active_channels = 16;  // {2,4,8,16,32}
  int m_group_sel_lsb  = 10;    // byte-address 기준 bit7 (trx 주소 기준으로 보정해 사용)

 public:
  void init() override { /* no-op */ }

  void setup(IFrontEnd* fe, IMemorySystem* ms) override {
    m_dram = ms->get_ifce<IDRAM>();

    const auto& count = m_dram->m_organization.count;
    m_num_levels = static_cast<int>(count.size());
    m_addr_bits.resize(m_num_levels);
    for (int lvl = 0; lvl < m_num_levels; ++lvl)
      m_addr_bits[lvl] = calc_log2(count[lvl]);

    // 컬럼 비트는 내부 prefetch(=BL)만큼 축소
    m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

    // 트랜잭션 바이트 수 = (prefetch words) × (channel width bits) / 8
    const int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
    m_tx_offset = calc_log2(tx_bytes);

    // 레벨 인덱스
    m_row_bits_idx = m_dram->m_levels("row");
    m_col_bits_idx = m_num_levels - 1;
    m_total_channels = count[0];

    // (원하면) YAML에서 m_active_channels / m_group_sel_lsb를 읽어와도 됨
  }

  void apply(Request& req) override {
    req.addr_vec.resize(m_num_levels, -1);

    // 1) 주소를 ‘트랜잭션 주소’로 정규화
    Addr_t a = req.addr >> m_tx_offset;

    // 2) 그룹/활성채널 계산
    const int C = m_total_channels;                               // 전체 채널 수(예: 32)
    const int A = std::max(1, std::min(m_active_channels, C));    // 활성 채널 수
    const int G = C / A;                                          // 그룹 개수(예: 32/16=2)

    // 3) Fig.3(c) 비트 선택: byte-addr 기준 LSB를 trx-addr 기준으로 보정
    const int group_bits   = (G > 1) ? calc_log2(G) : 0;
    const int group_lsb_trx = std::max(0, m_group_sel_lsb - static_cast<int>(m_tx_offset));
    const int group_id     = (group_bits > 0) ? ((a >> group_lsb_trx) & ((1 << group_bits) - 1)) : 0;

    // 4) 그룹 내 채널 오프셋 = 하위 log2(A) 비트
    const int intra_bits = (A > 1) ? calc_log2(A) : 0;
    const int intra_id   = (intra_bits > 0) ? (a & ((1 << intra_bits) - 1)) : 0;

    // 5) 최종 채널
    const int ch = group_id * A + intra_id;
    req.addr_vec[0] = ch;

    // ---- 디버그/히스토그램 (중복 선언 금지: 한 번만) ----
    static uint64_t hist[64] = {0};   // 32채널이면 0..31만 사용
    static uint64_t seen = 0;

    if (ch < 64) hist[ch]++;

    if (seen < 32) {
      spdlog::info(
        "[HBM4_GroupMapper] addr={:#x} a={} tx_off={} grp_lsb_trx={} grp_bits={} grp_id={} intra_bits={} intra_id={} ch={}",
        (unsigned long long)req.addr, (unsigned long long)a,
        (int)m_tx_offset, group_lsb_trx, group_bits, group_id,
        intra_bits, intra_id, ch);
    }

    if (++seen % 5000 == 0) {
      for (int i = 0; i < std::min(C, 32); ++i)
        spdlog::info("[ch-hist] ch{}={}", i, hist[i]);
    }
    // -----------------------------------------------

    // 6) 나머지 레벨은 RoBaRaCoCh 순서로 슬라이스
    Addr_t b = a;
    (void) slice_lower_bits(b, m_addr_bits[0]); // 채널 비트폭만큼 “소모”
    req.addr_vec[m_col_bits_idx] = slice_lower_bits(b, m_addr_bits[m_col_bits_idx]);
    for (int lvl = 1; lvl <= m_row_bits_idx; ++lvl) {
      req.addr_vec[lvl] = slice_lower_bits(b, m_addr_bits[lvl]);
    }
  }
};

} // namespace Ramulator
