// JTFRAME horizontal scaler, used by Groovy's "CRT scale" option.
// jtframe_hsize and jtframe_linebuf are from JTFRAME by Jose Tejada Gomez
// (https://github.com/jotego/jtcores, modules/jtframe), GPL-3.0-or-later.
// This file was missing from the Groovy repository, which kept the core from
// building. Changes for Groovy are marked with "Groovy:".

/* SPDX-FileCopyrightText: 2026 Jose Tejada Gomez
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Date: 1-8-2021 */

// Applies horizontal scaling to an analogue signal
  
module jtframe_hsize #( parameter
    COLORW     =4,    // bits per colour
    VW         =11    // line counter width. Groovy: raised from 9 so modes up to 2048 pixels per line (blanking included) fit
) (
    input              clk,
    input              pxl_cen,
    input              pxl2_cen,

    input        [3:0] scale,
    input        [4:0] offset,
    input              enable,

    input [COLORW-1:0] r_in,
    input [COLORW-1:0] g_in,
    input [COLORW-1:0] b_in,
    input              HS_in,
    input              VS_in,
    input              HB_in,
    input              VB_in,
    // filtered video
    output reg            HS_out,
    output reg            VS_out,
    output reg            HB_out,
    output reg            VB_out,
    output reg [COLORW-1:0] r_out,
    output reg [COLORW-1:0] g_out,
    output reg [COLORW-1:0] b_out
);

localparam SW = 8;

wire [   VW+SW-2:0] summand;
wire [COLORW*3-1:0] rgb_out, rgb_in;
reg  [        VW:0] rdcnt_l=0, rdcnt=0,rgbcnt_i=0;
reg  [      SW-2:0] rdfrac=0,rgbfrac=0;
reg  [      VW-1:0] hb0, hb1;
reg  [      VW-1:0] wrcnt, rgbcnt,  hmax;

reg  VSl, HSl, LHBl, LHBll, VBl, pass, over;

assign rgb_in  = {r_in, g_in, b_in};
assign summand = {{VW{1'b0}},~scale[3],{SW-6{scale[3]}},scale};

always @(posedge clk) if(pxl_cen) begin
    HSl    <=  HS_in;
    LHBl   <= ~HB_in;
    LHBll  <=  LHBl;
    HB_out <= HB_in;
    HS_out <= HS_in;
    VB_out <= VB_in;

    // VB must be adjusted to prevent the bottom line from being washed out
    if( ~HB_in & ~LHBl ) {VB_out, VBl} <= {VBl, VB_in};
    if(  HS_in & ~HSl ) begin
        wrcnt  <= 0;
        hmax   <= wrcnt;
        VSl    <= VS_in;        
        VS_out <= VSl;
    end else begin
        wrcnt  <= wrcnt + 1'd1;
    end
    if( !enable) begin 
        VB_out <= VB_in;
        VS_out <= VS_in;
    end
    // Register when HB toggles
    if(  LHBl   & ~LHBll  ) hb1 <= wrcnt;
    if(  HB_in  &  LHBl   ) hb0 <= wrcnt;
    // colour output
    {r_out,g_out,b_out} <= enable ? (!HB_out && pass ? rgb_out : {3*COLORW{1'b0}}) : rgb_in;
end

always @(posedge clk) if(pxl2_cen) begin
    {rdcnt,rdfrac} <= {rdcnt,rdfrac} + {1'b0,summand};
    pass           <= (rgbcnt <= hb0) && (rgbcnt >= hb1);
    if( ~HS_in &  HSl)     over <= 1;
    if(  HS_in & ~HSl & over) begin 
        {rdcnt,rdfrac} <= { {VW-4{offset[4]}}, offset,{SW-1{1'b0}} };
        rdcnt_l        <= rdcnt;
        rgbcnt         <= rgbcnt_i[VW-1:0]; 
        rgbfrac        <= 0;  
        rgbcnt_i       <= ({1'b0,hmax}-rdcnt_l)>>1;
        over           <= 0; //Avoids this step being done twice in the same HS
    end else begin 
        {rgbcnt,rgbfrac} <= {rgbcnt,rgbfrac} + summand;
    end
end

jtframe_linebuf #(.DW(COLORW*3), .AW(VW)) u_line(
    .clk      ( clk          ),
    .LHBL     (~HS_in        ),
    // Port 0: writes 
    .wr_data  ( rgb_in       ),
    .wr_addr  ( wrcnt ),
    .we       ( pxl_cen      ),
    // Port 1
    .rd_addr  (rgbcnt),
    .rd_gated (              ),
    .rd_data  ( rgb_out      )
    );
endmodule

module jtframe_linebuf #(parameter
    DW=8,
    AW=9
)(
    input            clk,
    input            LHBL,  // the line buffer is swapped when this signal goes low
                            // you can use ~HS instead of LHBL too
    // New data writes
    input   [AW-1:0] wr_addr,
    input   [DW-1:0] wr_data,
    input            we,
    // Old data reads
    input   [AW-1:0] rd_addr,
    output  [DW-1:0] rd_data,
    output  [DW-1:0] rd_gated
);

reg     line, last_LHBL;

wire [DW-1:0]       dump_data;

`ifdef SIMULATION
initial begin
    line = 0;
end
`endif

always @(posedge clk) begin
    last_LHBL <= LHBL;
    if( !LHBL && last_LHBL )
        line <= ~line;
end

assign rd_gated = LHBL ? rd_data : {DW{1'b0}};

jtframe_rpwp_ram #(.AW(AW+1),.DW(DW)) u_line(
    .clk    ( clk           ),

    .din    ( wr_data       ),
    .wr_addr( {line,wr_addr}),
    .we     ( we            ),

    .rd_addr({~line,rd_addr}),
    .dout   ( rd_data       )
);

endmodule

// Simple dual port RAM: one write port, one registered read port.
// Groovy: small stand-in for JTFRAME's jtframe_rpwp_ram so the core does not
// need the rest of the JTFRAME RAM library. Quartus infers block RAM from it.
module jtframe_rpwp_ram #(parameter DW=8, AW=10) (
    input               clk,
    input      [DW-1:0] din,
    input      [AW-1:0] wr_addr,
    input               we,
    input      [AW-1:0] rd_addr,
    output reg [DW-1:0] dout
);

reg [DW-1:0] mem[0:(2**AW)-1];

always @(posedge clk) begin
    dout <= mem[rd_addr];
    if( we ) mem[wr_addr] <= din;
end

endmodule
