set datafile sep ','
set terminal png size 1200.941 crop
set title 'Map algorithm'
set output 'plotMap.png'
set xlabel 'Y[mm]'
set ylabel 'X[mm]'
set xrange [-0.52:0.1]
set yrange [-1:0.5]
set palette rgb 33,12,10
plot 'mapData.csv' u 1:2:3:3 w p lt 7 ps var lc var, 'A2Data.csv' u 1:2:3:3 w p lt 7 ps var lc var
