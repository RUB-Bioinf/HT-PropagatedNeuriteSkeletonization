echo Running test-sh
sleep 5
whoami
chmod +x ../resources
chmod +x ../output
cd ../resources
find . -type d -print0 >../dirs.txt
cd ../output
mkdir -p "$(date +"%d-%m-%Y")"
cd "$(date +"%d-%m-%Y")"
mkdir -p "$(date +%R)"
cd "$(date +%R)"
xargs -0 mkdir -p <../../../dirs.txt
rm ../../../dirs.txt
echo finished running test-sh!
sleep 5