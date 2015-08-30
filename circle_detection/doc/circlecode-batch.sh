convert -size 598x398 xc:white -bordercolor black -border 1 a.png 
for i in $(seq 0 3);do 
	convert a.png -fill black -stroke black -draw "ellipse $((($i%2)*220+115+115*($i/2))),$((115+180*($i/2))) 90,90 0,360"  a.png;
done
a=0
	cp a.png b.png
	cp b.png a.png
	an=$(($1))
for i in $(seq 0 3);
do
	ax=$((an%4))
	an=$((an/4))
	ca=0
	sa=0
	if [ $(($ax%2)) == 0 ];
	then
		ca=$((2*($ax/2)-1))
	else
		sa=$((2*($ax/2)-1))
	fi
	convert a.png -stroke white -fill white -draw "ellipse $((($i%2)*220+115+115*($i/2)+$ca*20)),$(((115+180*($i/2))+$sa*20)) 35,35 0,360" a.png;
done
