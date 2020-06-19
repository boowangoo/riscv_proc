module and_assign ( x, y, z );
    input wire x, y;
    output wire z;

    assign z = x & y;
endmodule // and_assign

module reg_and
    (
    input wire clock,
    input wire reset,
    input wire in,
    output out
    );

    // Sequential logic
    reg out;

    always @(posedge clock ) begin
        if ( reset )
            out <= 0;
        else
            out <= in;
    end

    // assign out = out;
endmodule // reg_and

module dut;
    reg x, y, reset;
    reg clock = 1;
    wire z, out;

    initial begin
        $dumpfile("reg-and.vcd");
        // All variables (0) from module instance gate
        $dumpvars(0, gate);
        $dumpvars(0, reg_gate);

        #0 reset = 1;
        #20 x = 0; y = 0;
        reset <= 0; $display("Reset complete");
        #10 x = 1; y = 1; $display("set 1 1");
        #10 x = 1; y = 0;
        #10 x = 1; y = 1;
        #10 x = 0; y = 1;
        #20 $finish;
    end
    // Instantiate AND gate
    and_assign gate ( .x(x), .y(y), .z(z));
    reg_and reg_gate ( .clock(clock), .reset(reset), .in(z), .out(out) );
    
    // Toggle clock signal every 5
    always begin
        #5 clock = ~clock;
    end
    
    always @(posedge clock) begin
        $display("time=%t, reset=%b, x=%b, y=%b, z=%b, out=%b", $time,reset,x,y,z, out);
        #1 $display("time=%t, out2=%b", $time, out);
    end
endmodule // dut
