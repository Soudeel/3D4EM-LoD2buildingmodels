set mappinglib=D:\eos_mapping\Tools\bin
set lasFile=building
set lasExt=las
set shpFile=building


set path=%path%;%mappinglib%

lasduplicate -i %lasFile%.%lasExt% -o %lasFile%_thin.%lasExt%
las2txt -i %lasFile%_thin.%lasExt% -o %lasFile%.txt -parse xyz
ascii2laser -i %lasFile%.txt -odir .\ -root %lasFile% -x 1 -y 2 -z 3 -max 10000000
growsurfaces -i %lasFile%.laser -overwrite -seedradius 2.0 -growradius 1 -maxdistgrow 0.3 -minsegsize 30
shp2pcm -ishp %shpFile%.shp -op %shpFile%.objpts -ot %shpFile%.top
3DBuilding -il %lasFile%.laser -ip %shpFile%.objpts -it %shpFile%.top -op %shpFile%_bld.objpts -ot %shpFile%_bld.top
pcm2shp -ip %shpFile%_bld.objpts -it %shpFile%_bld.top -odir %shpFile%_bld.shp  -model
Evaluate3DBuilding -i %lasFile%.laser -iop %shpFile%_bld.Objpts -iot %shpFile%_bld.top -a 5.0 -r 0.3

pause