set terminal qt font 'Verdana,12'
set yrange [1:1e6]
set ylabel 'Number of Leaves'
set xlabel 'Topological Event'
set key top left
set logscale y
set title 'Number of Leaves in the Tree of Maps for '.ARG3 noenhanced

plot ARG1 using 1:3 with lines lt 5 lc rgb "#87ceeb" lw 2 title 'Exhaustive Search', \
     ARG2 using 1:3 with lines lt 7 lc rgb "#ff6347" lw 2 title 'Lazy Evaluation'

