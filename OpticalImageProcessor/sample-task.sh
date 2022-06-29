#!/bin/sh

# input files
PAN1=/Users/stone/Downloads/357444514_锄禾觅當午/给彭遒的泰景三号01星示例数据/20220316/KEL_MN200_PAN-1_20220316_317120309_00000001_00_12_B1.RAW
PAN2=/Users/stone/Downloads/357444514_锄禾觅當午/给彭遒的泰景三号01星示例数据/20220316/KEL_MN200_PAN-2_20220316_317120309_00000001_00_12_B1.RAW
MSS1=/Users/stone/Downloads/357444514_锄禾觅當午/给彭遒的泰景三号01星示例数据/20220316/KEL_MN200_MSS-1_20220316_317120309_00000001_00_12_B1.RAW
MSS2=/Users/stone/Downloads/357444514_锄禾觅當午/给彭遒的泰景三号01星示例数据/20220316/KEL_MN200_MSS-2_20220316_317120309_00000001_00_12_B1.RAW
RRC_PAN1=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/PAN-1.csv
RRC_PAN2=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/PAN-2.csv
RRC_MSS1B1=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-1.B1.csv
RRC_MSS1B2=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-1.B2.csv
RRC_MSS1B3=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-1.B3.csv
RRC_MSS1B4=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-1.B4.csv
RRC_MSS2B1=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-2.B1.csv
RRC_MSS2B2=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-2.B2.csv
RRC_MSS2B3=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-2.B3.csv
RRC_MSS2B4=/Users/stone/Library/CloudStorage/OneDrive-Personal/工作/星睿/产品_项目/星睿七号-泰景三号/相对辐射定标系数CSV/MSS-2.B4.csv

# generated files
S1_PAN1=`basename $(echo "${PAN1}" | tr '[:lower:]' '[:upper:]') .RAW`.RRC.RAW
S1_PAN2=`basename $(echo "${PAN2}" | tr '[:lower:]' '[:upper:]') .RAW`.RRC.PRESTT.RAW

ALGN_MSS1=`basename $(echo "${MSS1}" | tr '[:lower:]' '[:upper:]') .RAW`.ALIGNED.TIFF
ALGN_MSS2=`basename $(echo "${MSS2}" | tr '[:lower:]' '[:upper:]') .RAW`.ALIGNED.TIFF

STT_PAN=stitched-PAN.TIFF
STT_MSS=stitched-MSS.TIFF

# processing parameters
FOLDCOL_PAN=200
FOLDCOL_MSS=50
OIP=./OpticalImageProcessor

# run processing command
# step 1.
#    1.1 Calculate inter-CMOS pixel correlation parameter (for CMOS stitching use) from PAN raw images
#    1.2 Do Relative Radiometric Correction for each PAN raw images, and generate corresponding RRC-ed PAN image files
#    1.3 Do pixel correction for CMOS-2 PAN image according to the correlation result calculated in step 1.1
echo "STEP 1: prestitching ..."
${OIP} prestitch --pan1=${PAN1} --pan2=${PAN2} --rrc1=${RRC_PAN1} --rrc2=${RRC_PAN2}

if [ $? -eq 0 ]; then
    echo "OK."
else
    echo "ERROR! prestitching process failed!"
    exit 1
fi

# step 2. Stitch CMOS-1 & CMOS-2 PAN raw images
echo "STEP 2: PAN stitching ..."
${OIP} stitch --image1=${S1_PAN1} --image2=${S1_PAN2} --fold-cols=${FOLDCOL_PAN} -o ${STT_PAN}

if [ $? -eq 0 ]; then
    echo "OK, stitched PAN tiff image '${STT_PAN}' generated."
else
    echo "ERROR! PAN stitching failed!"
    exit 2
fi

#step 3.
#    3.1 generate inter-band pixel aligned MSS tiff image for CMOS-1
echo "STEP 3.1: inter-band pixel alignment for MSS of CMOS-1 ..."
${OIP} --pan=${S1_PAN1} --mss=${MSS1} \
    --rrc-msb1=${RRC_MSS1B1} \
    --rrc-msb2=${RRC_MSS1B2} \
    --rrc-msb3=${RRC_MSS1B3} \
    --rrc-msb4=${RRC_MSS1B4}

if [ $? -eq 0 ]; then
    echo "OK."
else
    echo "ERROR! Aligned CMOS-1 MSS TIFF generation failed!"
    exit 3
fi

#    3.2 generate inter-band pixel aligned MSS tiff image for CMOS-2
echo "STEP 3.2: inter-band pixel alignment for MSS of CMOS-2 ..."
${OIP} --pan=${S1_PAN2} --mss=${MSS2} \
    --rrc-msb1=${RRC_MSS2B1} \
    --rrc-msb2=${RRC_MSS2B2} \
    --rrc-msb3=${RRC_MSS2B3} \
    --rrc-msb4=${RRC_MSS2B4}

if [ $? -eq 0 ]; then
    echo "OK."
else
    echo "ERROR! Aligned CMOS-2 MSS TIFF generation failed!"
    exit 3
fi

# step 4. Stitch CMOS-1 & CMOS-2 MSS tiff images
echo "STEP 2: MSS stitching ..."
${OIP} stitch --image1=${ALGN_MSS1} --image2=${ALGN_MSS2} --fold-cols=${FOLDCOL_MSS} -o ${STT_MSS}

if [ $? -eq 0 ]; then
    echo "OK, stitched MSS tiff image '${STT_MSS}' generated."
else
    echo "ERROR! MSS stitching failed!"
    exit 4
fi

echo "All done."
