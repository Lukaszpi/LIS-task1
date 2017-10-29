T_ptgrey_imumav =  [0.005140507079658428, -0.9999738350881948, -0.005089629256733209, 0.03275991334837573; -0.9999835877268608, -0.005153299705934457, 0.0025035532859538833, -0.0871049399819889; -0.0025297161656549967, 0.005076676190956924, -0.9999839138181037, -0.12476284779815548; 0.0, 0.0, 0.0, 1.0]

get_q = @(T) sprintf('%d ', circshift(rotm2quat(T(1:3,1:3)).*[1 1 1 1], [0, 3]))  %calc quat and rearrange to x y z w from w x y z
get_d = @(T) sprintf('%d ', T(1:3,4)') % get distance


%output tf string
str='<node pkg="tf" type="static_transform_publisher" name="%s_position_broadcaster" args="%s %s $(arg mav_name)/base_link $(arg mav_name)/%s 100" /> \n';

fprintf(str, 'ptgrey', get_d(pinv(T_ptgrey_imumav)), get_q(pinv(T_ptgrey_imumav)), 'ptgrey');
1

