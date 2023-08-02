rm -rf upload_dir
mkdir upload_dir
cd upload_dir
git clone git@github.com:CentralLaserFacility/source-downloads.git -b master
git clone git@github.com:CentralLaserFacility/motor.git -b development_clf --recurse-submodules motor-R7-2-2

tar -czvf motor-R7-2-2.tar.gz motor-R7-2-2/

rm -rf motor-R7-2-2


mv source-downloads/EPICS/motor-R7-2-2.tar.gz source-downloads/EPICS/old_"`date +"%T%Z%d-%m-%Y"`"
mv motor-R7-2-2.tar.gz source-downloads/EPICS/

cd source-downloads/

git add --all
git commit -m "Updated motor-R7-2-2 on `date +"%T%Z %d-%m-%Y"`"
git push

rm -rf upload_dir
