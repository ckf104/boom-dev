//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// ICache
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.ifu

import chisel3._
import chisel3.util._
import chisel3.util.random._

import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.rocket.{HasL1ICacheParameters, ICacheParams, ICacheErrors, ICacheReq}




import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix}

/**
 * ICache module
 *
 * @param icacheParams parameters for the icache
 * @param hartId the id of the hardware thread in the cache
 * @param enableBlackBox use a blackbox icache
 */
class ICache(
  val icacheParams: ICacheParams,
  val staticIdForMetadataUseOnly: Int)(implicit p: Parameters)
  extends LazyModule
{
  lazy val module = new ICacheModule(this)
  val masterNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    sourceId = IdRange(0, icacheParams.fetchMSHRNum + icacheParams.pfMSHRNum),
    name = s"Core ${staticIdForMetadataUseOnly} ICache")))))

  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes
  private val wordBytes = icacheParams.fetchBytes
}

/**
 * IO Signals leaving the ICache
 *
 * @param outer top level ICache class
 */
class ICacheResp(val outer: ICache) extends Bundle
{
  val data = UInt((outer.icacheParams.fetchBytes*8).W)
  val replay = Bool()
  val ae = Bool()
}

/**
 * IO Signals for interacting with the ICache
 *
 * @param outer top level ICache class
 */
class ICacheBundle(val outer: ICache) extends BoomBundle()(outer.p)
  with HasBoomFrontendParameters
{
  val req = Flipped(Decoupled(new ICacheReq))
  val s1_paddr = Input(UInt(paddrBits.W)) // delayed one cycle w.r.t. req

  val s1_kill = Input(Bool()) // delayed one cycle w.r.t. req
  val s2_kill = Input(Bool()) // delayed two cycles; prevents I$ miss emission

  val resp = Valid(new ICacheResp(outer))
  val invalidate = Input(Bool())

  val perf = Output(new Bundle {
    val acquire = Bool()
  })

  //Enable_PerfCounter_Support
  val icache_valid_access = Output(Bool())

  // Prefetch redirect bookkeeping. These signals may clear the registered s2
  // prefetch state, but must not gate the current-cycle prefetch MSHR issue path.
  val mshr_flush    = Input(Bool()) // predecode / backend flush
  val bpd_f3_flush  = Input(Bool())
  val bpd_f3_ftq_idx = Input(new FTQPtr)

  // s0: prefetch virtual address for tag lookup
  val s0_pf_valid   = Input(Bool())
  val s0_pf_vaddr   = Input(UInt(vaddrBitsExtended.W))
  val s0_pf_blocked = Output(Bool())
  val s0_pf_dist_bucket = Input(UInt(3.W))

  // s1: prefetch physical address from TLB and control
  val s1_pf_valid      = Input(Bool())
  val s1_pf_ppc        = Input(UInt(paddrBits.W))
  val s1_pf_ppc_valid  = Input(Bool())
  val s1_pf_ppc_exp    = Input(Bool())
  val s1_pf_clear      = Input(Bool())
  val s1_pf_ftq_idx    = Input(new FTQPtr)
  // ICache accepted this s1 prefetch this cycle (advanced to internal s2)
  val s1_pf_can_advance    = Output(Bool())

  // Prefetch refill distance bucket: valid when a prefetch MSHR completes writeback
  // and the writeback is not cancelled by fencei
  val pf_refill_dist_bucket = Valid(UInt(3.W))

  // From frontend: s2 pipeline successfully enters f3 (not flushed)
  val f3_enq_fire = Input(Bool())
  // Prefetch accuracy: true when a prefetch-filled line is first accessed by IFU
  val pf_hit_success = Output(Bool())
}

/**
 * Get a tile-specific property without breaking deduplication
 */
object GetPropertyByHartId
{
  def apply[T <: Data](tiles: Seq[RocketTileParams], f: RocketTileParams => Option[T], hartId: UInt): T = {
    PriorityMux(tiles.collect { case t if f(t).isDefined => (t.tileId.U === hartId) -> f(t).get })
  }
}

// Copied from XiangShan ICacheMissUnit.scala
class MSHRResp(implicit p: Parameters) extends CoreBundle with HasBoomFrontendParameters {
  val paddr  :  UInt = UInt(paddrBits.W)
  val tag:      UInt = UInt(tagBits.W)
  val vSetIdx:  UInt = UInt(idxBits.W)
  val way:      UInt = UInt(wayBits.W)
  val distBucket: UInt = UInt(3.W)
}

class LookUpMSHR(implicit p: Parameters) extends CoreBundle with HasBoomFrontendParameters {
  val paddr: UInt                = UInt(paddrBits.W)
  val hit:   Bool                = Input(Bool())
}

class ICacheMSHRIO(implicit p: Parameters) extends CoreBundle with HasBoomFrontendParameters {
  val fencei:    Bool                       = Input(Bool())
  val flush:     Bool                       = Input(Bool())
  val invalid:   Bool                       = Input(Bool())
  val req:       DecoupledIO[UInt]          = Flipped(DecoupledIO(UInt(paddrBits.W)))
  val reqDistBucket: UInt                   = Input(UInt(3.W))
  val acquire:   DecoupledIO[UInt]          = DecoupledIO(UInt(paddrBits.W))
  val lookUps:   Vec[LookUpMSHR]            = Flipped(Vec(2, new LookUpMSHR))
  val resp:      Valid[MSHRResp]            = ValidIO(new MSHRResp)
  val victimWay: UInt                       = Input(UInt(wayBits.W))

  val addr_valid_debug: Bool = Output(Bool())
  val mshr_valid_debug: Bool = Output(Bool())
}

// MSHR 与 flush 的交互
//   * 在 MSHR 发起 l2 请求之前，发生 flush，是否要 invalid 该 MSHR，在请求 fire 的同一周期呢？
//   * 在 l2 请求 fire 了之后，发生 flush，是否要 invalid 该写回？
// BOOM 的行为
//   * 忽略重定向造成的 flush，接受 fence.i 造成的 flush
//   * 在 MSHR 发起 l2 请求之前或请求 fire 的同一周期，忽略拉高的 fence.i 信号
//   * 在 l2 请求 fire 之后的 flush 信号会 invalid 后续写回（即使最后一个写回周期和 fence.i 同时拉高）
// 新引入的 MSHR 尽可能与 BOOM 保持一致：
//   * 在 fence.i 拉高时，prefetch MSHR 和 fetch MSHR 都会被 invalid。如果 l2 请求已经 fire 了,
//     会 invalid 后续写回
//   * fetch MSHR 不响应重定向造成的 flush
//   * 如果重定向造成的 flush 在 prefetch MSHR 发起 l2 请求之前或请求 fire 的同一周期, invalid 该 MSHR,
//     否则忽略该 flush 信号（因为对应的 cache line 大概率已经 invalid 掉了，写回不亏）

// 时序上：
//   * 预期的时序：s2 周期检查 mshr hit，入 mshr 的同时就可以发起 l2 请求（即 flow 为 true）
//   * mshr 处于 invalid 状态时才接受新请求（即 pipe 为 false）

// 冒险：在 s1 周期和 s2 周期我们都需要检查 MSHR hit，避免出现向下级 cache 请求已经写回的 cache line
// s1 周期我们检查 refill_done，看它与 io.s1_paddr 是否匹配，如果匹配则认为 MSHR hit
// s2 周期我们检查每个 MSHR 的 paddr 与 s2_paddr 是否匹配，如果匹配则认为 MSHR hit。如果 flush 或
// fence.i 同时拉高呢？这也无所谓，首先 fetch MSHR 不响应 flush，而 prefetch MSHR 遇到 flush 的话，
// prefetch 流水线应该同步清空。而 fence.i 本身就会 invalidate MSHR（这里的关键在于，误判为 MSHR hit
// 是安全的，因为 fetch 这边会 replay，重复请求。而误判为 MSHR miss 会导致对同一个 cache line 发起多
// 个 MSHR 请求，导致 ICache 中包含重复表项，这是危险的）

// TODO: 目前的实现是，当 isFetch 为 true 时，行为与 BOOM 保持一致；当 isFetch 为 false 时，模块仍保留
// flush 输入，但 ICache 只用 fence.i/invalidate 取消 prefetch MSHR。f3 redirect 不再 flush 已经进入
// prefetch MSHR 发请求路径的请求；错误预取只影响带宽/污染，不影响架构正确性。
class ICacheMSHR(isFetch: Boolean, ID: Int)(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters
{
  val io: ICacheMSHRIO = IO(new ICacheMSHRIO)

  private val valid = RegInit(Bool(), false.B)
  // this MSHR doesn't respond to fetch and sram
  private val flush  = RegInit(Bool(), false.B)
  private val fencei = RegInit(Bool(), false.B)
  // this MSHR has been issued
  private val issue = RegInit(Bool(), false.B)

  private val blkPaddr = RegInit(UInt((paddrBits - blockOffBits).W), 0.U)
  private val way      = RegInit(UInt(wayBits.W), 0.U)
  private val distBucket = RegInit(UInt(3.W), 0.U)

  // look up and return result at the same cycle
  for (i <- 0 until 2) {
    if (isFetch) {
      io.lookUps(i).hit := valid && !fencei && (io.lookUps(i).paddr(paddrBits-1, blockOffBits) === blkPaddr)
    } else {
      io.lookUps(i).hit := valid && !fencei && (io.lookUps(i).paddr(paddrBits-1, blockOffBits) === blkPaddr)
    }
  }

  // invalid when the req hasn't been issued
  if (isFetch) {
    when (io.fencei) {
      when(!issue) {
        valid := false.B
      } .otherwise {
        fencei := true.B
      }
    }
  } else {
    when(io.fencei) {
      when(!issue) {
        valid := false.B
      } .otherwise {
        fencei := true.B
      }
    }
    when (io.flush) {
      when (!issue) {
        valid := false.B
      }
    }
  }


  // receive request and register
  if (isFetch) {
    io.req.ready := !valid && !io.fencei
  } else {
    io.req.ready := !valid && !io.flush && !io.fencei
  }
  when(io.req.fire) {
    valid    := true.B
    flush    := false.B
    issue    := false.B
    fencei   := false.B
    blkPaddr := io.req.bits(paddrBits-1, blockOffBits)
    distBucket := io.reqDistBucket
  }

  // send request to L2
  if (isFetch) {
    io.acquire.valid := (valid || io.req.fire) && !issue && !io.fencei
  } else {
    io.acquire.valid := (valid || io.req.fire) && !issue && !io.flush && !io.fencei
  }
  io.acquire.bits := Cat(Mux(valid, blkPaddr, io.req.bits(paddrBits-1, blockOffBits)), 0.U(blockOffBits.W))

  // get victim way when acquire fire
  when(io.acquire.fire) {
    issue := true.B
    way   := io.victimWay
  }

  // invalid request when grant finish
  when(io.invalid) {
    valid  := false.B
    issue  := false.B
    fencei := false.B
    if (!isFetch) {
      flush  := false.B
    }
  }

  // offer the information other than data for write sram and response fetch
  if (isFetch) {
    io.resp.valid       := valid && !fencei
  } else {
    io.resp.valid       := valid && !fencei
  }
  io.resp.bits.paddr    := Cat(blkPaddr, 0.U(blockOffBits.W))
  io.resp.bits.tag      := blkPaddr(tagBits+idxBits-1,idxBits)
  io.resp.bits.vSetIdx  := blkPaddr(idxBits-1, 0)
  io.resp.bits.way      := way
  io.resp.bits.distBucket := distBucket

  require(tagBits + idxBits == blkPaddr.getWidth)

  io.addr_valid_debug := issue
  io.mshr_valid_debug := valid
  if (IN_SIMULATION) {
    // free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)

    // plusarg to enable MSHR debug prints
    val mshr_printf = PlusArg("mshr-printf", 0, "print icache mshr debug info", 1)

    when (mshr_printf(0)) {
      // 1) New request accepted into this MSHR
      when (io.req.fire) {
        val req_paddr = Cat(io.req.bits(paddrBits-1, blockOffBits), 0.U(blockOffBits.W))
        printf(p"[${cycleCount} ICacheMSHR Req] id=${ID} paddr=${Hexadecimal(req_paddr)}\n")
      }

      // 2) TL request (acquire) fired to lower level cache
      when (io.acquire.fire) {
        val acq_paddr = io.acquire.bits
        printf(p"[${cycleCount} ICacheMSHR Acquire] id=${ID} paddr=${Hexadecimal(acq_paddr)}\n")
      }

      // 3) Prefetch MSHR invalidated by flush before being issued
      if (!isFetch) {
        when (io.flush && valid && !issue) {
          val line_paddr = Cat(blkPaddr, 0.U(blockOffBits.W))
          printf(p"[${cycleCount} ICacheMSHR Flush] id=${ID} paddr=${Hexadecimal(line_paddr)}\n")
        }
      }

      // 4) fence.i observed by this MSHR (may cancel writeback if already issued)
      when (io.fencei) {
        val line_paddr = Cat(blkPaddr, 0.U(blockOffBits.W))
        printf(p"[${cycleCount} ICacheMSHR FenceI] id=${ID} paddr=${Hexadecimal(line_paddr)} issued=${issue}\n")
      }

      // 5) MSHR invalidated after grant/line refill completes
      when (io.invalid) {
        val line_paddr = Cat(blkPaddr, 0.U(blockOffBits.W))
        printf(p"[${cycleCount} ICacheMSHR Invalidate] id=${ID} paddr=${Hexadecimal(line_paddr)} cancelled=${fencei}\n")
      }
    }

    when (io.invalid) {
      assert (valid, p"ICache invalidating an invalid MSHR (ID = $ID)\n")
      assert (issue, p"ICache invalidating a non-issued MSHR (ID = $ID)\n")
    }
    when (!valid) {
      assert (!issue, p"ICache MSHR (ID = $ID) is not valid but has been issued\n")
      assert (!fencei, p"ICache MSHR (ID = $ID) is not valid but has fencei set\n")
    }
    when (!issue) {
      assert (!fencei, p"ICache MSHR (ID = $ID) is not issued but has fencei set\n")
    }
  }
}
// TODO: 
//   * 检查 MSHR 写回和流水线读 Tag Array 的冒险，避免为已经 hit 的 cache line 再发起 l2 请求
//   * 检查 fence.i 会 invalid 所有的 in-flight 读写和 cache line
// TODO: 校验 prefetch MSHR 的实现, 尤其是重定向的 flush，我觉得 s1 flush，s2 flush 和 MSHR flush
// 需要分别看待

/**
 * Main ICache module
 *
 * @param outer top level ICache class
 */
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val enableICacheDelay = tileParams.core.asInstanceOf[BoomCoreParams].enableICacheDelay
  val io = IO(new ICacheBundle(outer))
  val (tl_out, edge_out) = outer.masterNode.out(0)

  require(isPow2(nSets) && isPow2(nWays))
  require(usingVM)
  require(pgIdxBits >= untagBits)

  // How many bits do we intend to fetch at most every cycle?
  val wordBits = outer.icacheParams.fetchBytes*8
  // Require fixed 64-bit fetch width with single bank
  require (wordBits == 64 && nBanks == 1, s"wordBits must be 64 and nBanks must be 1, got wordBits=$wordBits nBanks=$nBanks")
  // TL bus width can be 64 bits (same as fetch) or 128 bits (wide refill)
  val tlDataBits = tl_out.d.bits.data.getWidth
  require (tlDataBits == wordBits || tlDataBits == 2 * wordBits,
    s"TL data width must be $wordBits or ${2*wordBits}, got $tlDataBits")
  // Ensure refillCycles (derived from rowBits) is consistent with TL bus width
  require (tlDataBits * refillCycles == cacheBlockBytes * 8,
    s"TL data width ($tlDataBits) * refillCycles ($refillCycles) must equal block bits (${cacheBlockBytes*8})")
  // When TL bus is wider than fetch width, we split each data SRAM into two sub-banks
  val refillIsWide = (tlDataBits == 2 * wordBits)



  val s0_valid = io.req.fire
  val s0_vaddr = io.req.bits.addr

  val s1_valid = RegNext(s0_valid)
  val s1_tag_hit = Wire(Vec(nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_)
  val s2_valid = RegNext(s1_valid && !io.s1_kill)
  val s2_hit = RegNext(s1_hit)
  val s2_paddr = RegNext(io.s1_paddr)

  // Prefetch pipeline inputs
  val s0_pf_valid = io.s0_pf_valid
  val s0_pf_vaddr = io.s0_pf_vaddr
  val s1_pf_valid = io.s1_pf_valid
  val s1_pf_ppc   = io.s1_pf_ppc
  val s1_pf_ppc_valid = io.s1_pf_ppc_valid
  
  val s1_pf_fire  = Wire(Bool())

  // Pipeline prefetch distance bucket: s0 -> s1 -> s2_pf
  val s1_pf_dist_bucket = RegNext(io.s0_pf_dist_bucket)
  val s2_pf_dist_bucket = RegEnable(s1_pf_dist_bucket, s1_pf_fire)
  
  val s1_pf_MSHR_hit  = Wire(Bool())
  val s1_pf_cache_hit = Wire(Bool())
  val s2_pf_valid        = RegEnable(s1_pf_valid && s1_pf_ppc_valid && !io.s1_pf_clear && !io.s1_pf_ppc_exp,
                                    false.B, s1_pf_fire)
  val s2_pf_hit          = RegEnable(s1_pf_MSHR_hit || s1_pf_cache_hit, s1_pf_fire)
  val s2_pf_ppc          = RegEnable(s1_pf_ppc, s1_pf_fire)
  val s2_pf_ftq_idx      = RegEnable(io.s1_pf_ftq_idx, s1_pf_fire)
  val s2_pf_clear        = io.bpd_f3_flush && s2_pf_valid && s2_pf_ftq_idx > io.bpd_f3_ftq_idx
  when (s2_pf_clear || io.mshr_flush) {
    s2_pf_valid := false.B
  }

  val nFetchMSHRs = outer.icacheParams.fetchMSHRNum
  val nPrefetchMSHRs = outer.icacheParams.pfMSHRNum

  val fetchMSHRs = Seq.tabulate(nFetchMSHRs) { i =>
    Module(new ICacheMSHR(isFetch = true, ID = i))
  }
  val pfMSHRs = Seq.tabulate(nPrefetchMSHRs) { i =>
    Module(new ICacheMSHR(isFetch = false, ID = nFetchMSHRs + i))
  }

  private val allMSHRs = fetchMSHRs ++ pfMSHRs
  private val nMSHRs = allMSHRs.length

  val s1_MSHR_hit = Wire(Bool())
  val s2_MSHR_hit_vec = VecInit(allMSHRs.map(_.io.lookUps(0).hit))
  val s2_MSHR_hit = s2_MSHR_hit_vec.asUInt.orR
  val access_hit = s2_hit || s2_MSHR_hit || RegNext(s1_MSHR_hit)

  // Connect common control signals to all fetch/prefetch MSHRs
  fetchMSHRs.foreach { m =>
    m.io.fencei := io.invalidate
    m.io.flush  := false.B
    m.io.lookUps(0).paddr := s2_paddr
    m.io.lookUps(1).paddr := s2_pf_ppc
    m.io.reqDistBucket := 0.U
  }
  pfMSHRs.foreach { m =>
    m.io.fencei := io.invalidate
    // Do not let f3 branch-prediction redirects suppress prefetch MSHR issue.
    // Wrong-path prefetches are harmless for correctness; fence.i/invalidate
    // remains the correctness mechanism that can cancel them.
    m.io.flush  := false.B
    m.io.lookUps(0).paddr := s2_paddr
    m.io.lookUps(1).paddr := s2_pf_ppc
    m.io.reqDistBucket := s2_pf_dist_bucket
  }

  // Allocate miss requests into the first available MSHR
  val need_mshr = s2_valid && !access_hit && !io.s2_kill
  var alloc_taken = false.B
  for (i <- 0 until nFetchMSHRs) {
    val can_alloc = need_mshr && fetchMSHRs(i).io.req.ready && !alloc_taken
    fetchMSHRs(i).io.req.valid := can_alloc
    fetchMSHRs(i).io.req.bits  := s2_paddr
    alloc_taken = alloc_taken || can_alloc
  }

  val mshr_resp_valids = VecInit(allMSHRs.map(_.io.resp.valid))
  val mshr_resp_paddrs = VecInit(allMSHRs.map(_.io.resp.bits.paddr))
  val mshr_resp_tags   = VecInit(allMSHRs.map(_.io.resp.bits.tag))
  val mshr_resp_idxs   = VecInit(allMSHRs.map(_.io.resp.bits.vSetIdx))
  val mshr_resp_ways   = VecInit(allMSHRs.map(_.io.resp.bits.way))
  val mshr_resp_dist_buckets = VecInit(allMSHRs.map(_.io.resp.bits.distBucket))

  val refill_fire = tl_out.a.fire
  val refill_one_beat = tl_out.d.fire && edge_out.hasData(tl_out.d.bits)
  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
  val refill_done = refill_one_beat && d_done

  val refill_src = tl_out.d.bits.source
  val refill_src_oh = UIntToOH(refill_src, nMSHRs)

  val refill_resp_valid = Mux1H(refill_src_oh, mshr_resp_valids)
  val invalidated = !refill_resp_valid
  val refill_paddr = Mux1H(refill_src_oh, mshr_resp_paddrs)
  val refill_tag   = Mux1H(refill_src_oh, mshr_resp_tags)
  val refill_idx   = Mux1H(refill_src_oh, mshr_resp_idxs)
  val refill_dist_bucket = Mux1H(refill_src_oh, mshr_resp_dist_buckets)

  // Output prefetch refill distance bucket: valid when a prefetch MSHR
  // completes writeback and its resp is not cancelled by fencei
  val is_pf_refill = refill_src >= nFetchMSHRs.U
  io.pf_refill_dist_bucket.valid := refill_done && !invalidated && is_pf_refill
  io.pf_refill_dist_bucket.bits  := refill_dist_bucket

  io.req.ready := !refill_one_beat
  //Enable_PerfCounter_Support
  io.icache_valid_access := s2_valid
  // Block prefetch s0 when a refill write is occurring to the tag array
  io.s0_pf_blocked := refill_done

  tl_out.d.ready := true.B
  require (edge_out.manager.minLatency > 0)

  val initial_fire = RegInit(true.B)
  initial_fire := false.B
  val victim_way = if (isDM) 0.U else LFSR(16, refill_fire || initial_fire)(log2Ceil(nWays)-1,0)
  val repl_way = Mux1H(refill_src_oh, mshr_resp_ways)
  allMSHRs.foreach(_.io.victimWay := victim_way)
  for (i <- 0 until nFetchMSHRs) {
    fetchMSHRs(i).io.invalid := refill_done && (refill_src === i.U)
  }
  for (i <- 0 until nPrefetchMSHRs) {
    pfMSHRs(i).io.invalid := refill_done && (refill_src === (nFetchMSHRs + i).U)
  }
  s1_MSHR_hit := refill_done && !invalidated && refill_paddr(paddrBits-1, blockOffBits) ===
                  io.s1_paddr(paddrBits-1, blockOffBits)

  // Prefetch s1-level hit: cache hit or in-flight refill to same block.
  // These s1 results will be registered and used in the s2_pf stage so that
  // prefetch has the same two-stage pipeline structure as the fetch path.
  val s1_pf_tag_only_hit = Wire(Vec(nWays, Bool()))
  val s1_pf_vb_hit = Wire(Vec(nWays, Bool()))
  val s1_pf_tag_hit_vec = VecInit(Seq.tabulate(nWays) { i => s1_pf_vb_hit(i) && s1_pf_tag_only_hit(i) })
  s1_pf_cache_hit := s1_pf_tag_hit_vec.asUInt.orR
  s1_pf_MSHR_hit  := refill_done && !invalidated &&
                      (refill_paddr(paddrBits-1, blockOffBits) === s1_pf_ppc(paddrBits-1, blockOffBits))

  // Conflict with IFU s2 request to the same block
  val s2_blkPaddr = s2_paddr(paddrBits-1, blockOffBits)
  val s2_pf_blkPaddr = s2_pf_ppc(paddrBits-1, blockOffBits)

  // Register s1 prefetch results into s2_pf stage so that prefetch has
  // the same two-stage pipeline structure as fetch. The s2_pf stage then
  // decides whether a new prefetch MSHR is needed and drives s1_pf_advance.
  val s2_pf_cache_hit    = RegNext(s1_pf_cache_hit)
  val s2_pf_MSHR_hit_vec = VecInit(allMSHRs.map(_.io.lookUps(1).hit))
  val s2_pf_MSHR_hit     = s2_pf_MSHR_hit_vec.asUInt.orR
  val s2_pf_access_hit   = s2_pf_MSHR_hit || s2_pf_hit
  val s2_pf_conflict_with_ifu_s2 = s2_valid && (s2_pf_blkPaddr === s2_blkPaddr)

  // Decide in s2_pf whether prefetch needs a new MSHR entry
  val need_pf_mshr = s2_pf_valid && !s2_pf_access_hit && !s2_pf_conflict_with_ifu_s2

  // Allocate prefetch requests into the first available prefetch MSHR (s2_pf)
  if (nPrefetchMSHRs > 0) {
    val pf_ready_vec  = VecInit(pfMSHRs.map(_.io.req.ready)).asUInt
    val pf_alloc_mask = PriorityEncoderOH(pf_ready_vec)

    for (i <- 0 until nPrefetchMSHRs) {
      pfMSHRs(i).io.req.valid := need_pf_mshr && pf_alloc_mask(i)
      pfMSHRs(i).io.req.bits  := s2_pf_ppc
    }
  }

  // Signal to frontend whether the prefetch that was in s1 has now been
  // fully consumed/handled by ICache (either hit/duplicate or MSHR-allocated)
  val s2_pf_consumed = if (nPrefetchMSHRs > 0) {
    val pf_ready_vec = VecInit(pfMSHRs.map(_.io.req.ready)).asUInt
    !need_pf_mshr || pf_ready_vec.orR
  } else {
    true.B
  }

  s1_pf_fire   := !s2_pf_valid || s2_pf_consumed
  if (fullMSHRSkip) {
    io.s1_pf_can_advance := true.B
  } else {
    io.s1_pf_can_advance := s1_pf_fire
  }

  val tag_array = SyncReadMem(nSets, Vec(nWays, UInt(tagBits.W)))
  // IFU tag read port
  val tag_rdata = tag_array.read(s0_vaddr(untagBits-1, blockOffBits), !refill_done && s0_valid)
  // Prefetch tag read port (second independent read port), blocked only when refill writes
  val pf_tag_rdata = tag_array.read(s0_pf_vaddr(untagBits-1, blockOffBits), !refill_done && s0_pf_valid)
  when (refill_done) {
    tag_array.write(refill_idx, VecInit(Seq.fill(nWays)(refill_tag)), Seq.tabulate(nWays)(repl_way === _.U))
  }

  val vb_array = RegInit(0.U((nSets*nWays).W))
  when (refill_one_beat) {
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
  }

  when (io.invalidate) {
    vb_array := 0.U
  }

  // Prefetch accuracy tracking arrays
  // pf_array: marks lines filled by prefetch MSHRs
  // visit_array: marks lines that have been successfully accessed by IFU (entered f3)
  val pf_array    = RegInit(0.U((nSets*nWays).W))
  val visit_array = RegInit(0.U((nSets*nWays).W))

  // Update pf_array on refill completion
  when (refill_done && !invalidated) {
    pf_array := pf_array.bitSet(Cat(repl_way, refill_idx), is_pf_refill)
  }
  when (io.invalidate) {
    pf_array := 0.U
  }

  val s2_dout   = Wire(Vec(nWays, UInt(wordBits.W)))
  val s1_bankid = Wire(Bool())

  val s1_tag_only_hit = Wire(Vec(nWays, Bool()))
  val s1_vb_hit = Wire(Vec(nWays, Bool()))

  for (i <- 0 until nWays) {
    val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
    val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
    s1_vb_hit(i) := vb_array(Cat(i.U, s1_idx))
    val tag = tag_rdata(i)
    s1_tag_only_hit(i) := tag === s1_tag
    s1_tag_hit(i) := s1_vb_hit(i) && s1_tag_only_hit(i)

    // Prefetch tag/VB hits (use pf tag read port and pf paddr)
    val pf_s1_idx = s1_pf_ppc(untagBits-1, blockOffBits)
    val pf_s1_tag = s1_pf_ppc(tagBits+untagBits-1, untagBits)
    s1_pf_vb_hit(i) := vb_array(Cat(i.U, pf_s1_idx))
    val pf_tag = pf_tag_rdata(i)
    s1_pf_tag_only_hit(i) := pf_tag === pf_s1_tag
  }

  dontTouch(s1_tag_only_hit)
  dontTouch(s1_vb_hit)
  dontTouch(s1_tag_hit)
  dontTouch(s1_pf_tag_only_hit)
  dontTouch(s1_pf_vb_hit)

  val ramDepth = nSets * refillCycles

  if (!refillIsWide) {
    // TL width == fetch width (64 bits). Single SRAM per way.
    val dataArrays = (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayWay_${x}",
        desc = "ICache Data Array",
        size = ramDepth,
        data = UInt(wordBits.W)
      )
    }
    s1_bankid := 0.U
    for ((dataArray, i) <- dataArrays.zipWithIndex) {
      def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
      val s0_ren = s0_valid

      val wen = (refill_one_beat && !invalidated) && repl_way === i.U

      val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
                    row(s0_vaddr))
      when (wen) {
        dataArray.write(mem_idx, tl_out.d.bits.data)
      }
      if (enableICacheDelay)
        s2_dout(i) := dataArray.read(RegNext(mem_idx), RegNext(!wen && s0_ren))
      else
        s2_dout(i) := RegNext(dataArray.read(mem_idx, !wen && s0_ren))
    }
  } else {
    // TL width (128 bits) is twice the fetch width (64 bits).
    // Split each way into two sub-banks (B0 = lower 64 bits, B1 = upper 64 bits)
    // so that each 128-bit TL beat can be written in one cycle.
    val dataArraysB0 = (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayB0Way_${x}",
        desc = "ICache Data Array",
        size = ramDepth,
        data = UInt(wordBits.W)
      )
    }
    val dataArraysB1 = (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayB1Way_${x}",
        desc = "ICache Data Array",
        size = ramDepth,
        data = UInt(wordBits.W)
      )
    }

    // s0_vaddr bit log2(wordBits/8) = bit 3 selects which 64-bit half within
    // a 128-bit TL beat: 0 -> B0 (lower), 1 -> B1 (upper)
    s1_bankid := RegNext(s0_vaddr(log2Ceil(wordBits/8)))

    for (i <- 0 until nWays) {
      // Row address: {set_index, beat_within_line}
      def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
      val s0_ren = s0_valid
      val wen = (refill_one_beat && !invalidated) && repl_way === i.U

      // Both sub-banks share the same row index
      val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
                    row(s0_vaddr))

      // Write: split 128-bit TL data into two 64-bit halves
      when (wen) {
        val data = tl_out.d.bits.data
        dataArraysB0(i).write(mem_idx, data(wordBits-1, 0))
        dataArraysB1(i).write(mem_idx, data(2*wordBits-1, wordBits))
      }

      // Read: read both sub-banks, select based on s2_bankid
      if (enableICacheDelay) {
        val b0_data = dataArraysB0(i).read(RegNext(mem_idx), RegNext(!wen && s0_ren))
        val b1_data = dataArraysB1(i).read(RegNext(mem_idx), RegNext(!wen && s0_ren))
        s2_dout(i) := Mux(RegNext(RegNext(s0_vaddr(log2Ceil(wordBits/8)))), b1_data, b0_data)
      } else {
        val b0_data = dataArraysB0(i).read(mem_idx, !wen && s0_ren)
        val b1_data = dataArraysB1(i).read(mem_idx, !wen && s0_ren)
        s2_dout(i) := Mux(RegNext(s1_bankid), RegNext(b1_data), RegNext(b0_data))
      }
    }
  }
  val s2_tag_hit = RegNext(s1_tag_hit)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  val s2_bankid = RegNext(s1_bankid)
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)

  // Prefetch accuracy: compute s2 line address and check pf/visit arrays
  val s2_idx = s2_paddr(untagBits-1, blockOffBits)
  val s2_line_addr = Cat(s2_hit_way, s2_idx)

  // Successful prefetch: first IFU access (entered f3) to a prefetch-filled line
  val s2_pf_hit_prefetched = io.f3_enq_fire && s2_hit &&
    pf_array(s2_line_addr) && !visit_array(s2_line_addr)
  io.pf_hit_success := s2_pf_hit_prefetched

  // Update visit_array: set on successful f3 entry, clear on refill replacement
  val visit_set_mask = Mux(io.f3_enq_fire && s2_hit,
    1.U((nSets*nWays).W) << s2_line_addr, 0.U((nSets*nWays).W))
  val visit_clr_mask = Mux(refill_done,
    1.U((nSets*nWays).W) << Cat(repl_way, refill_idx), 0.U((nSets*nWays).W))
  visit_array := (visit_array | visit_set_mask) & ~visit_clr_mask
  when (io.invalidate) {
    visit_array := 0.U
  }

  // s2_dout already contains the correct 64-bit data (bank-selected in refillIsWide case)
  val s2_data = s2_way_mux

  io.resp.bits.ae := DontCare
  io.resp.bits.replay := DontCare
  io.resp.bits.data := s2_data
  io.resp.valid := s2_valid && s2_hit

  // TL-A: arbitrate among fetch and prefetch MSHRs and encode MSHR ID into source field
  val fetch_acq_arb = Module(new Arbiter(UInt(paddrBits.W), nFetchMSHRs))
  for (i <- 0 until nFetchMSHRs) {
    fetch_acq_arb.io.in(i) <> fetchMSHRs(i).io.acquire
  }
  val fetch_acq_valid = fetch_acq_arb.io.out.valid
  val fetch_acq_bits  = fetch_acq_arb.io.out.bits
  val fetch_acq_id    = fetch_acq_arb.io.chosen

  // --- FIFO-based prefetch acquire scheduling (replaces Arbiter) ---
  // A flow-capable Queue records the order in which prefetch MSHRs are allocated.
  // The head of the Queue selects which prefetch MSHR's acquire signals to present.
  // Dequeue when the selected acquire fires, or when the head entry is stale
  // (MSHR was cancelled by flush/fencei before being issued).

  val pfIdxWidth = if (nPrefetchMSHRs > 1) log2Ceil(nPrefetchMSHRs) else 1

  // Enqueue interface: which prefetch MSHR was just allocated this cycle?
  val pfFifoEnq = Wire(Decoupled(UInt(pfIdxWidth.W)))
  if (nPrefetchMSHRs > 0) {
    val pf_fire_vec = VecInit(pfMSHRs.map(_.io.req.fire))
    pfFifoEnq.valid := pf_fire_vec.asUInt.orR
    pfFifoEnq.bits  := PriorityEncoder(pf_fire_vec)  // at most one fires per cycle
  } else {
    pfFifoEnq.valid := false.B
    pfFifoEnq.bits  := 0.U
  }

  // Flow Queue: when empty and enqueue fires, data is immediately visible at
  // dequeue in the same cycle. Flushed only on fence.i/invalidate; f3 redirects
  // must not suppress an already allocated prefetch MSHR's acquire path.
  val pfFifoDeq = Queue(pfFifoEnq, math.max(nPrefetchMSHRs, 1), flow = true,
                         flush = Some(io.invalidate))

  val pf_effective_idx = pfFifoDeq.bits

  // Mux the selected prefetch MSHR's acquire signals
  val pf_acq_valid_vec = VecInit(
    if (nPrefetchMSHRs > 0) pfMSHRs.map(_.io.acquire.valid)
    else Seq(false.B)
  )
  val pf_acq_bits_vec = VecInit(
    if (nPrefetchMSHRs > 0) pfMSHRs.map(_.io.acquire.bits)
    else Seq(0.U(paddrBits.W))
  )
  val pf_acq_valid_raw = if (nPrefetchMSHRs > 0) {
    pf_acq_valid_vec(pf_effective_idx)
  } else {
    false.B
  }
  val pf_acq_bits      = pf_acq_bits_vec(pf_effective_idx)
  val pf_acq_id        = pf_effective_idx
  val pf_acq_valid     = pf_acq_valid_raw && (nPrefetchMSHRs > 0).B

  val choose_fetch = fetch_acq_valid
  val choose_pf    = !choose_fetch && pf_acq_valid
  val any_valid    = choose_fetch || choose_pf

  tl_out.a.valid := any_valid
  fetch_acq_arb.io.out.ready := tl_out.a.ready && choose_fetch

  // Drive ready back to the selected prefetch MSHR
  if (nPrefetchMSHRs > 0) {
    for (i <- 0 until nPrefetchMSHRs) {
      pfMSHRs(i).io.acquire.ready := tl_out.a.ready && choose_pf && pf_effective_idx === i.U
    }
  }

  // Dequeue from Queue: on acquire fire (TL-A accepts the prefetch request)
  // Stale entries (MSHR cancelled by flush/fencei) cannot occur without a
  // simultaneous Queue flush, so no explicit stale-drain logic is needed.
  pfFifoDeq.ready := tl_out.a.ready && choose_pf

  val pf_acq_id_ext = Wire(UInt(log2Ceil(math.max(nMSHRs, 1)).W))
  pf_acq_id_ext := pf_acq_id

  val acq_addr = Mux(choose_fetch, fetch_acq_bits, pf_acq_bits)
  val acq_source = Wire(UInt(log2Ceil(math.max(nMSHRs, 1)).W))
  acq_source := Mux(choose_fetch,
    fetch_acq_id,
    (nFetchMSHRs.U + pf_acq_id_ext))
  tl_out.a.bits := edge_out.Get(
    fromSource = acq_source,
    toAddress = acq_addr,
    lgSize = lgCacheBlockBytes.U)._2
  tl_out.b.ready := true.B
  tl_out.c.valid := false.B
  tl_out.e.valid := false.B

  io.perf.acquire := tl_out.a.fire

  // Printf
  if (IN_SIMULATION) {
    // free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)

    // plusarg to enable ICache debug prints
    val icache_printf = PlusArg("icache-printf", 0, "print icache debug info", 1)

    // Debug vectors for MSHR issue/valid/address
    val mshr_issue_vec      = VecInit(allMSHRs.map(_.io.addr_valid_debug))
    val mshr_issue_vec_prev = RegNext(mshr_issue_vec)
    val mshr_valid_vec      = VecInit(allMSHRs.map(_.io.mshr_valid_debug))

    // Track previous cycle TL-A fire and its corresponding MSHR ID
    val a_fire_prev     = RegNext(tl_out.a.fire, init = false.B)
    val acq_source_prev = RegNext(acq_source)

    // 1) When TL-A fires, the chosen MSHR's issue must be false in this cycle
    when (tl_out.a.fire) {
      assert(!mshr_issue_vec(acq_source),
        p"ICache MSHR ${acq_source} issue must be false when TL-A request fires")
    }

    // 2) On the cycle after TL-A fire, only the chosen MSHR may toggle issue 0->1
    when (a_fire_prev) {
      for (i <- 0 until nMSHRs) {
        val prev_issue = mshr_issue_vec_prev(i)
        val curr_issue = mshr_issue_vec(i)
        when (i.U === acq_source_prev) {
          assert(!prev_issue && curr_issue,
            "ICache chosen MSHR issue must toggle from false to true after TL-A fire")
        } .otherwise {
          assert(!( !prev_issue && curr_issue),
            "ICache non-chosen MSHR issue must not toggle from false to true after TL-A fire")
        }
      }
    }

    // 3) All valid MSHRs must have unique request addresses
    for (i <- 0 until nMSHRs) {
      for (j <- i + 1 until nMSHRs) {
        when (mshr_valid_vec(i) && mshr_valid_vec(j)) {
          assert(mshr_resp_paddrs(i) =/= mshr_resp_paddrs(j),
            "ICache valid MSHRs must not have duplicate request addresses")
        }
      }
    }

    // Prefetch FIFO / MSHR consistency: if the selected prefetch MSHR has
    // acquire.valid = true, then there must be a corresponding FIFO entry.
    if (nPrefetchMSHRs > 0) {
      when (pf_acq_valid_vec(pf_effective_idx)) {
        assert(pfFifoDeq.valid,
          "ICache prefetch FIFO deq.valid must be true when selected MSHR acquire.valid is true")
      }
    }

    when (icache_printf(0)) {
      // 1) ICache sends miss request to lower level (TL-A channel)
      when (tl_out.a.fire) {
        val req_paddr = (tl_out.a.bits.address >> blockOffBits) << blockOffBits
        printf(p"[${cycleCount} ICache Req] mshr=${acq_source} paddr=${Hexadecimal(req_paddr)}\n")
      }

      // 2) Cache line refill completes
      when (refill_done) {
        val line_paddr = (refill_paddr >> blockOffBits) << blockOffBits
        printf(p"[${cycleCount} ICache Refill Done] mshr=${refill_src} paddr=${Hexadecimal(line_paddr)} " +
          p"set=${Hexadecimal(refill_idx)} way=${Hexadecimal(repl_way)}, cancelled=${invalidated}\n")
      }

      // 3) Tag hit in ICache
      when (s1_valid && s1_hit && PopCount(s1_tag_hit) > 1.U) {
        printf(p"[${cycleCount} ICache Hit] paddr=${Hexadecimal(io.s1_paddr)} s1_tag_hit=${Binary(s1_tag_hit.asUInt)}\n")
      }
    }
  }

  assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid || io.s1_kill,
    "Multiple ICache ways hit in s1")
  assert(!s1_valid || io.s1_kill || io.s1_paddr(pgIdxBits-1,0) === RegNext(io.req.bits.addr(pgIdxBits-1,0)),
    "s1_paddr does not match request address")
  val mshr_addr_valids = VecInit(allMSHRs.map(_.io.addr_valid_debug))
  val cur_mshr_addr_valid = Mux1H(refill_src_oh, mshr_addr_valids)
  assert(!refill_one_beat || cur_mshr_addr_valid,
    "ICache refill without a valid MSHR")

  if (IN_SIMULATION) {
    // Assertions on TL-D source range and non-interleaving between MSHRs.
    val d_inflight = RegInit(false.B)
    val d_source_reg = Reg(UInt(tl_out.d.bits.source.getWidth.W))
    // Track previous resp.valid value for the in-flight line's MSHR so that
    // we can detect false->true transitions while the line is in flight
    // (true->false is allowed for fence.i / flush).
    val d_prev_resp_valid = RegInit(false.B)

    when (refill_one_beat) {
      // 1) TL-D source must be within valid MSHR ID range.
      assert(refill_src < nMSHRs.U,
        "ICache TL-D source out of range of MSHRs")

      // 2) While a line refill is in flight, all data beats must
      //    come from the same source until refill_done is true.
      when (!d_inflight) {
        d_inflight := true.B
        d_source_reg := refill_src
        d_prev_resp_valid := refill_resp_valid
      } .otherwise {
        assert(refill_src === d_source_reg,
          "ICache TL-D source interleaving between MSHRs within a single line refill")

        // 3) For the MSHR corresponding to this source, resp.valid must not
        //    switch from false to true while this line refill is in flight.
        assert(!(d_prev_resp_valid === false.B && refill_resp_valid === true.B),
          "ICache MSHR resp.valid changed from false to true during an in-flight line refill")

        d_prev_resp_valid := refill_resp_valid
      }

      when (refill_done) {
        d_inflight := false.B
      }
    }
  }

  override def toString: String = BoomCoreStringPrefix(
    "==L1-ICache==",
    "Fetch bytes   : " + cacheParams.fetchBytes,
    "Block bytes   : " + (1 << blockOffBits),
    "vaddr bits    : " + vaddrBits,
    "vaddr ext bits: " + vaddrBitsExtended,
    "min pg level  : " + minPgLevels,
    "pg level      : " + pgLevels,
    "Row bytes     : " + rowBytes,
    "Word bits     : " + wordBits,
    "Sets          : " + nSets,
    "Ways          : " + nWays,
    "Refill cycles : " + refillCycles,
    "Bus width     : " + tlDataBits,
    "RAMs          : (" + wordBits + " x " + ramDepth + ") x " + (if (refillIsWide) 2 else 1) + " sub-bank(s) per way",
    "" + (if (refillIsWide) "Wide-refill (2 sub-banks)" else "Single-bank"),
    "I-TLB ways    : " + cacheParams.nTLBWays + "\n")
}
