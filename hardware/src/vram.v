module vram (
    input              clk,
    input       [18:0] ra0,
    output reg  [31:0] rd0,
    input       [18:0] ra1,
    output reg  [31:0] rd1,

    input       [18:0] wa,
    input       [31:0] wd,
    input              we);
    
    reg [31:0] ram [2^19-1:0];

    always @(posedge clk) begin
        rd0 <= ram[ra0];
        rd1 <= ram[ra1];

        if (we) ram[wa] <= wd;
    end

endmodule
