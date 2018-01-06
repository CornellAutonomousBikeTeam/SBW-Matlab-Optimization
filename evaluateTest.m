function score=evaluateTest(d,x,y,xB,yB,currentSegment)

%to evaluate how well the bike navigates
    
    xC=x(currentSegment);
    xD=x(currentSegment+1);
    yC=y(currentSegment);
    yD=y(currentSegment+1);
    
    error=sum(d.^2);
    
    for i=1:currentSegment-1
    traveled=sum((x(i+1)-x(i)).^2+(y(i)-y(i+1)).^2);
    end    
    
    pathLength=sqrt((xD-xC)^2+(yD-yC)^2);
    unitPath=[xD-xC,yD-yC,0]/pathLength;
    bikeLoc=[xB-xC,yB-yC,0];
    extra=dot(unitPath,bikeLoc);
    
    traveled=traveled+extra;
    
    score=traveled-2*error;
    
end


