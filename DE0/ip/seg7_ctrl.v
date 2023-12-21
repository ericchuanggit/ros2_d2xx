module seg7_ctrl (
    input [3:0] iDIG,
    output reg [6:0] oSEG
);

    always @(iDIG) begin
        case(iDIG)
            4'b0000: oSEG = 7'b1000000;
            4'b0001: oSEG = 7'b1111001;
            4'b0010: oSEG = 7'b0100100;
            4'b0011: oSEG = 7'b0110000;
            4'b0100: oSEG = 7'b0011001;
            4'b0101: oSEG = 7'b0010010;
            4'b0110: oSEG = 7'b0000010;
            4'b0111: oSEG = 7'b1111000;
            4'b1000: oSEG = 7'b0000000;
            4'b1001: oSEG = 7'b0010000;
            4'b1010: oSEG = 7'b0001000;
            4'b1011: oSEG = 7'b0000011;
            4'b1100: oSEG = 7'b1000110;
            4'b1101: oSEG = 7'b0100001;
            4'b1110: oSEG = 7'b0000110;
            4'b1111: oSEG = 7'b0001110;
            default: oSEG = 7'b1111111;  // Default case, all segments off
        endcase
    end

endmodule
