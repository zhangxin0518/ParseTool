#!/bin/bash

rm -rf data
mkdir data
cd data
mkdir img
mkdir map_img
cd ..

rm -rf data_sheet
mkdir data_sheet

echo "decompress the data"
cd build/
./ParseTool ../log/tcpStore.data

cd ../data/img/
ls > temp.txt
sed 's/.png//' temp.txt > times.txt
rm temp.txt
cp times.txt ../
rm times.txt

a=0
for i in *.png; do
  new=$(printf "%06d.png" "$a") #04 pad to length of 4
  mv -i -- "$i" "$new"
  let a=a+1
done





