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
    sourceId = IdRange(0, 1 + icacheParams.prefetch.toInt), // 0=refill, 1=hint
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


/**
 * Main ICache module
 *
 * @param outer top level ICache class
 */
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
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


  val invalidated = Reg(Bool())
  val refill_valid = RegInit(false.B)
  val refill_fire = tl_out.a.fire
  val s2_miss = s2_valid && !s2_hit && !RegNext(refill_valid)
  val refill_paddr = RegEnable(io.s1_paddr, s1_valid && !(refill_valid || s2_miss))
  val refill_tag = refill_paddr(tagBits+untagBits-1,untagBits)
  val refill_idx = refill_paddr(untagBits-1,blockOffBits)
  val refill_one_beat = tl_out.d.fire && edge_out.hasData(tl_out.d.bits)

  io.req.ready := !refill_one_beat
  //Enable_PerfCounter_Support
  io.icache_valid_access := s2_valid

  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
  val refill_done = refill_one_beat && d_done
  tl_out.d.ready := true.B
  require (edge_out.manager.minLatency > 0)

  val repl_way = if (isDM) 0.U else LFSR(16, refill_fire)(log2Ceil(nWays)-1,0)

  val tag_array = SyncReadMem(nSets, Vec(nWays, UInt(tagBits.W)))
  val tag_rdata = tag_array.read(s0_vaddr(untagBits-1, blockOffBits), !refill_done && s0_valid)
  when (refill_done) {
    tag_array.write(refill_idx, VecInit(Seq.fill(nWays)(refill_tag)), Seq.tabulate(nWays)(repl_way === _.U))
  }

  val vb_array = RegInit(0.U((nSets*nWays).W))
  when (refill_one_beat) {
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
  }

  when (io.invalidate) {
    vb_array := 0.U
    invalidated := true.B
  }

  val s2_dout   = Wire(Vec(nWays, UInt(wordBits.W)))
  val s1_bankid = Wire(Bool())

  for (i <- 0 until nWays) {
    val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
    val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
    val s1_vb = vb_array(Cat(i.U, s1_idx))
    val tag = tag_rdata(i)
    s1_tag_hit(i) := s1_vb && tag === s1_tag
  }
  assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid)

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

  // s2_dout already contains the correct 64-bit data (bank-selected in refillIsWide case)
  val s2_data = s2_way_mux

  io.resp.bits.ae := DontCare
  io.resp.bits.replay := DontCare
  io.resp.bits.data := s2_data
  io.resp.valid := s2_valid && s2_hit

  tl_out.a.valid := s2_miss && !refill_valid && !io.s2_kill
  tl_out.a.bits := edge_out.Get(
    fromSource = 0.U,
    toAddress = (refill_paddr >> blockOffBits) << blockOffBits,
    lgSize = lgCacheBlockBytes.U)._2
  tl_out.b.ready := true.B
  tl_out.c.valid := false.B
  tl_out.e.valid := false.B

  io.perf.acquire := tl_out.a.fire

  when (!refill_valid) { invalidated := false.B }
  when (refill_fire) { refill_valid := true.B }
  when (refill_done) { refill_valid := false.B }

  override def toString: String = BoomCoreStringPrefix(
    "==L1-ICache==",
    "Fetch bytes   : " + cacheParams.fetchBytes,
    "Block bytes   : " + (1 << blockOffBits),
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


