pathDatasetMoon='./dataset/SePT' #Example, it is necesary to change it by the dataset path

# #------------------------------------
# The full demo on Moon sequence:
echo "Launching SePTX with Stereo camera"
./Examples/Stereo/stereo_SePT EAO ./Vocabulary/ORBvoc.bin ./Examples/Stereo/SePT01.yaml "$pathDatasetMoon"/SePT01 ./Examples/Stereo/TimeStamps/SePT01.txt