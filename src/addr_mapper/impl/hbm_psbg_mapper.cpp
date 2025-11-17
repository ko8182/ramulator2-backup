// hbm_psbg_mapper.cpp
#include <stdexcept>
#include <vector>
#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"
#include <iostream>
namespace Ramulator {

// HBM: channel, pseudochannel, bankgroup, bank, row, column
class HBM_PsBg_RoBaRaCoCh : public IAddrMapper, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IAddrMapper, HBM_PsBg_RoBaRaCoCh,
      "HBM_PsBg_RoBaRaCoCh",
      "HBM mapping with explicit pseudochannel/bankgroup in RoBaRaCoCh bit-slicing")

 private:
  IDRAM* m_dram = nullptr;

  // 지연 초기화 상태
  bool m_ready = false;
  int m_num_levels = -1;
  std::vector<int> m_addr_bits;   // 각 레벨 비트폭
  Addr_t m_tx_offset = -1;        // 내부 프리패치*채널폭 만큼 건너뛸 하위비트

  // 레벨 인덱스
  int m_idx_channel = -1;
  int m_idx_pch     = -1; // pseudochannel (옵션)
  int m_idx_bg      = -1; // bankgroup    (옵션)
  int m_idx_bank    = -1;
  int m_idx_row     = -1;
  int m_idx_col     = -1;

  void ensure_initialized() {
  if (m_ready) return;
  if (!m_dram)
    throw std::runtime_error("HBM_PsBg_RoBaRaCoCh: DRAM nullptr");

  // DRAM init() 후에 organization.count 가 levels 개수만큼 채워져 있어야 함
  const auto& count = m_dram->m_organization.count;
  if (count.empty())
    throw std::runtime_error("HBM_PsBg_RoBaRaCoCh: DRAM organization not ready; init order issue");

  // ✅ levels 크기와 count 크기 일치 확인
  int expect_levels = (int)m_dram->m_levels.size();
  if ((int)count.size() != expect_levels) {
    throw std::runtime_error(
      "HBM_PsBg_RoBaRaCoCh: organization.count size mismatch. "
      "count.size=" + std::to_string(count.size()) +
      " vs levels.size=" + std::to_string(expect_levels) +
      " (Did DRAM init() run? Did you pick the intended DRAM impl in YAML?)"
    );
  }

  // 레벨별 비트폭 계산
  m_num_levels = (int)count.size();
  m_addr_bits.assign(m_num_levels, 0);
  for (int i = 0; i < m_num_levels; i++) {
    m_addr_bits[i] = calc_log2(count[i]);
  }

  // 레벨 인덱스 조회 (이 시점에는 LUT 준비됨)
  auto get_level = [&](const char* name) -> int {
    try { return m_dram->m_levels(name); }
    catch (...) { return -1; }
  };
  m_idx_channel = get_level("channel");
  m_idx_pch     = get_level("pseudochannel"); // 없으면 -1
  m_idx_bg      = get_level("bankgroup");     // 없으면 -1
  m_idx_bank    = get_level("bank");
  m_idx_row     = get_level("row");
  m_idx_col     = get_level("column");

  if (m_idx_channel < 0 || m_idx_bank < 0 || m_idx_row < 0 || m_idx_col < 0) {
    throw std::runtime_error(
      "HBM_PsBg_RoBaRaCoCh: required levels missing (channel/bank/row/column)"
    );
  }

  // column 레벨은 내부 prefetch만큼 하위비트가 흡수됨
  {
    int p = calc_log2(m_dram->m_internal_prefetch_size);
    if (m_addr_bits[m_idx_col] >= p) m_addr_bits[m_idx_col] -= p;
    else m_addr_bits[m_idx_col] = 0;
  }

  // 전송 단위(tx) 하위비트 스킵: prefetch * channel_width/8
  {
    int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
    m_tx_offset = calc_log2(tx_bytes);
  }

  // (선택) 디버그: 필요할 때만 켜세요
  // #include <iostream> 필요
  // std::cout << "[HBM_PsBg] idx: "
  //           << "ch=" << m_idx_channel
  //           << " pch=" << m_idx_pch
  //           << " bg=" << m_idx_bg
  //           << " bank=" << m_idx_bank
  //           << " row=" << m_idx_row
  //           << " col=" << m_idx_col << "\n";
  // std::cout << "[HBM_PsBg] addr_bits:";
  // for (int i = 0; i < m_num_levels; i++) std::cout << " " << m_addr_bits[i];
  // std::cout << " (num_levels=" << m_num_levels << ")\n";
  // std::cout << "[HBM_PsBg] org.count:";
  // for (auto v : count) std::cout << " " << v;
  // std::cout << "  dq=" << m_dram->m_organization.dq
  //           << "  ch_width=" << m_dram->m_channel_width
  //           << "  prefetch=" << m_dram->m_internal_prefetch_size << "\n";

  m_ready = true;
}
 public:
  void init() override {} // 여기서는 아무 것도 하지 않음

  void setup(IFrontEnd* /*fe*/, IMemorySystem* ms) override {
    // 여기선 포인터만 잡고, 실제 초기화는 apply() 첫 호출 때
    m_dram = ms->get_ifce<IDRAM>();
  }

  void apply(Request& req) override {
    ensure_initialized();

    req.addr_vec.assign(m_num_levels, -1);

    // 전송단위만큼 하위 비트를 먼저 제거
    Addr_t addr = req.addr >> m_tx_offset;

    // RoBaRaCoCh 순으로 비트를 잘라 “해당 레벨 인덱스”에 배치
    // 1) Channel
    req.addr_vec[m_idx_channel] = slice_lower_bits(addr, m_addr_bits[m_idx_channel]);

    // 2) Row
    req.addr_vec[m_idx_row] = slice_lower_bits(addr, m_addr_bits[m_idx_row]);

    // 3) Bank
    req.addr_vec[m_idx_bank] = slice_lower_bits(addr, m_addr_bits[m_idx_bank]);

    // 4) BankGroup (optional)
    if (m_idx_bg >= 0 && m_addr_bits[m_idx_bg] > 0)
      req.addr_vec[m_idx_bg] = slice_lower_bits(addr, m_addr_bits[m_idx_bg]);

    // 5) PseudoChannel (optional)
    if (m_idx_pch >= 0 && m_addr_bits[m_idx_pch] > 0)
      req.addr_vec[m_idx_pch] = slice_lower_bits(addr, m_addr_bits[m_idx_pch]);

    // 6) Column (마지막)
    req.addr_vec[m_idx_col] = slice_lower_bits(addr, m_addr_bits[m_idx_col]);
  }
};

} // namespace Ramulator

