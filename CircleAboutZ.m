function CP=CircleAboutZ(r)
%for use in Kate/Diego's animation, that I am using to test my controllers

%   r   radius of the circle
%
%   C=[x_row,y_row,z_row];

theta=linspace(0,2*pi,100);

x=r*cos(theta);
y=r*sin(theta);

CP=[x;y;zeros(size(x))];

end