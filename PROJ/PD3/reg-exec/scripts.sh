case $1 in
    synth)  # synth
        iverilog -g 2005 -o reg-exec reg-exec.v
        ;;
    run)    # run
        vvp reg-exec
        ;;
    wave)   # wave
        gtkwave reg-exec-SumArray.vcd
        ;;
esac
