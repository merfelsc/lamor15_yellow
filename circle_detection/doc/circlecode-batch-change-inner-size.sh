convert -size 748x498 xc:white -bordercolor black -border 1 a.png 

num_circle=5;
r_outer=90;
r_inner_max=40;
r_inner_min=20;
r_inner_del=$((($r_inner_max-$r_inner_min)/($num_circle-1)));

# Black circles
for i in $(seq 0 $(($num_circle-1)));do 
	convert a.png -fill black -stroke black -draw "ellipse $((($i%3)*220+115+115*($i/3))),$((115+180*($i/3))) $r_outer,$r_outer 0,360"  a.png;
done

an=$(($1))
	
# White circles
for i in $(seq 0 $(($num_circle-1)));do 
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
	r_inner=$(($r_inner_min+$r_inner_del*i));
	echo $r_inner
	convert a.png -stroke white -fill white -draw "ellipse $((($i%3)*220+115+115*($i/3)+$ca*20)),$(((115+180*($i/3))+$sa*20)) $r_inner,$r_inner 0,360" a.png;
done
