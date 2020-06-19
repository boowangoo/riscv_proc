case $1 in
    synth)  # synth
        iverilog -g 2005 -o mem-wb mem-wb.v
        ;;
    run)    # run
        vvp mem-wb
        ;;
    wave)   # wave
        gtkwave mem-wb-SumArray.vcd
        ;;
esac
