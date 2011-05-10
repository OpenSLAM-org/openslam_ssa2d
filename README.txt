#COMPILATION:
#
# SORRY FOR THE INCONVINENCE TO COMPILE G2O and SSA seperately
# We will work on that ;)

cd EXTERNAL/g2o/build
cmake ../
make
cd ../../../build
cmake ../
make

