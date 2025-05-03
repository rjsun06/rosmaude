rm actuator.pipe
pkill python3
pkill sensor-bin
pkill actuator-bin
./sensor-bin data/s1_sit.txt 1234 &
./sensor-bin data/p1.txt 1235 &
./actuator-bin actuator.pipe &
for i in {0..4}; do
    gnome-terminal --title "machine #$i" -- bash -c "python3 ./main.py n$i; exec bash" &
done
gnome-terminal --title "machine #5" -- bash -c "python3 ./main.py n5 --gui; exec bash" &