// read_file_ex.v
// note that, we need to create Modelsim project to run this file,
// or provide full path to the input-file i.e. adder_data.txt  

`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps

module read_file_ex;
    
    reg a, b;
    // sum_expected, carry_expected are merged together for understanding
    reg[1:0] sum_carry_expected; 

    // [3:0] = 4 bit data
    // [0:5] = 6 rows  in the file adder_data.txt
    reg[3:0] read_data [0:5];
    integer i;

    initial
    begin 
        // readmemb = read the binary values from the file
        // other option is 'readmemh' for reading hex values
        // create Modelsim project to use relative path with respect to project directory
        $readmemb("adder_data.txt", read_data);
        // or provide the compelete path as below
        // $readmemb("D:/Testbences/input_output_files/adder_data.txt", read_data);

        // total number of lines in adder_data.txt = 6
        for (i=0; i<6; i=i+1)
        begin
            // 0_1_0_1 and 0101 are read in the same way, i.e.
            //a=0, b=1, sum_expected=0, carry_expected=0 for above line;
            // but use of underscore makes the values more readable.
            {a, b, sum_carry_expected} = read_data[i]; // use this or below
            // {a, b, sum_carry_expected[0], sum_carry_expected[1]} = read_data[i];
            $display("%4b", read_data[i]);
            #20;  // wait for 20 clock cycle
        end
    end
endmodule
