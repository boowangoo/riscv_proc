from os import listdir, system
from os.path import isfile, join

import re

path1 = "../../rv32-benchmarks-master/simple-programs/"
path2 = "../../rv32-benchmarks-master/individual-instructions/"


files1 = [f for f in listdir(path1) if isfile(join(path1, f))]
xfiles1 = list(filter(lambda f: re.search(".*\.x", f), files1))

files2 = [f for f in listdir(path2) if isfile(join(path2, f))]
xfiles2 = list(filter(lambda f: re.search(".*\.x", f), files2))

with open("./main-mem.v", "r") as f1:
    verilog_file = f1.readlines()

    for i in range(len(xfiles1)):
        with open(path1 + xfiles1[i], "r") as f2:
            
            
            content = f2.readlines()
            verilog_file[0] = "`define FILE_PATH \"" +  path1 + xfiles1[i] + "\"\n"
            verilog_file[1] = "`define FILE_LINES " + str(len(content)) + "\n"
            verilog_file[2] = "`define VCD \"main-mem-" + xfiles1[i][:-2] + ".vcd\"\n"

            output_verilog_file = "main-mem" + xfiles1[i][:-2] + ".v"
            output_verilog_prog = "main-mem" + xfiles1[i][:-2]
            with open(output_verilog_file, "w+") as f3:
                for vl in verilog_file:
                    f3.write(vl)
                    
            system("echo " + "===========================")
            system("echo " + xfiles1[i][:-2])
            system("echo " + "===========================")
            system("iverilog -o " + output_verilog_prog + " " + output_verilog_file)
            system("vvp " + output_verilog_prog)
            
    for i in range(len(xfiles2)):
        with open(path2 + xfiles2[i], "r") as f2:

            content = f2.readlines()
            verilog_file[0] = "`define FILE_PATH \"" +  path2 + xfiles2[i] + "\"\n"
            verilog_file[1] = "`define FILE_LINES " + str(len(content)) + "\n"
            verilog_file[2] = "`define VCD \"main-mem-" + xfiles2[i][:-2] + ".vcd\"\n"

            output_verilog_file = "main-mem" + xfiles2[i][:-2] + ".v"
            output_verilog_prog = "main-mem" + xfiles2[i][:-2]
            with open(output_verilog_file, "w+") as f3:
                for vl in verilog_file:
                    f3.write(vl)
            
            system("echo " + "===========================")
            system("echo " + xfiles2[i][:-2])
            system("echo " + "===========================")
            system("iverilog -o " + output_verilog_prog + " " + output_verilog_file)
            result = system("vvp " + output_verilog_prog)
