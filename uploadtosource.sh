rm -rf upload_dir
mkdir upload_dir
cd upload_dir
git clone git@github.com:CentralLaserFacility/motor.git -b development_clf --recurse-submodules motor-R7-2-2

tar -czvf motor-R7-2-2.tar.gz motor-R7-2-2/

rm -rf motor-R7-2-2

git clone https://github.com/CentralLaserFacility/source-downloads -b master

mv source-downloads/EPICS/test source-downloads/EPICS/test_"`date +"%d-%m-%Y"`"
mv motor-R7-2-2.tar.gz source-downloads/EPICS/

cd source-downloads/

git add --all
git commit -m "Updated motor-R7-2-2 on `date +"%d-%m-%Y"`"
git push

rm -rf upload_dir
