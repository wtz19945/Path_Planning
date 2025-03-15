op = swf_obs_pos(1:2);
cp = [actual_foot_x(1);actual_foot_y(1)];

disp('section1')
vec = [-1;1]/norm([-1;1]);
inter = op + swf_xy_r(1) * vec;
cp(1) - op(1)
-cp(2) + op(2)
-dot(vec, cp - inter)

disp('section2')
vec = [-1;-1]/norm([-1;-1]);
inter = op + swf_xy_r(1) * vec;
cp(1) - op(1)
cp(2) - op(2)
-dot(vec, cp - inter)

disp('section3')
vec = [1;1]/norm([1;1]);
inter = op + swf_xy_r(1) * vec;
-cp(1) + op(1)
-cp(2) + op(2) 
-dot(vec, cp - inter) 

disp('section4')
vec = [1;-1]/norm([1;-1]);
inter = op + swf_xy_r(1) * vec;
-cp(1) + op(1)
cp(2) - op(2)
-dot(vec, cp - inter)



%%
th = 15 * 3.14 / 180;
rotA = [cos(th) -sin(th);sin(th) cos(th)];
rotB = [cos(th) sin(th);-sin(th) cos(th)];

vec = [-1;1]/norm([-1;1]);
rotB * vec
vec = [-1;-1]/norm([-1;-1]);
rotA * vec
vec = [1;1]/norm([1;1]);
rotA * vec
vec = [1;-1]/norm([1;-1]);
rotB * vec