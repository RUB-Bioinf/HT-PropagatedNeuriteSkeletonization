echo Running preprocessing
sleep 5
whoami
chmod +x ../resources
chmod +x ../output
cd ../resources
find . -type d -print0 > /out-tree/dirs.txt
cd ../output
mkdir -p "$(date +"%d-%m-%Y")"
cd "$(date +"%d-%m-%Y")"
mkdir -p "$(date "+%H-%M")"
cd "$(date "+%H-%M")"
xargs -0 mkdir -p < /out-tree/dirs.txt
rm /out-tree/dirs.txt
echo finished running test-sh!
sleep 5