set datafile sep ','
set terminal png size 1200.941 crop
set title 'Actuator force'
set output 'plot.png'
set xlabel 'Inclination[°]'
set ylabel 'Force[N]'
plot 'force.csv'
