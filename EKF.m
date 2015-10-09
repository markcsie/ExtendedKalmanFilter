function [ output ] = EKF( odomFileName, laserFileName, lineFileName, pointFileName, scanNum )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


odomInput = load(odomFileName);
laserInput = load(laserFileName);
mapLine = load(lineFileName);
mapPoint = load( pointFileName );
[ mapLineLength mapLineWidth ] = size( mapLine );
[ laserLength laserWidth ] = size( laserInput );
robotLine = zeros( laserLength, 2 );

rad = -135:0.3515625:135;
rad = rad * pi / 180;

robotPos = [ 1.6428; 0.1208; -0.1026 ];
robotCovar = [ 0.01, 0, 0; 0, 0.01, 0; 0, 0, 0.01 ];
robotMea = zeros( 2, 1 );
%on global coordinate
Q = [ 0.1, 0; 0, 0.7 ];
%on robot coordinate
R = [ 2, 0; 0, 2 ];

robotTra = zeros( laserLength, 2 );


%%%%%%%
drawMat = zeros( 2, 1 );
%%%%%%%


num = 0;
threshold = 0.08;
assThreshold = 2;



%EKF

for i = 1 : scanNum
    
    num = 0;
    laserMat = zeros( laserWidth - 2 ,2 );
    
    for j = 3 : laserWidth
        if laserInput( i, j ) < 20 || laserInput( i, j ) > 20000
            continue;
        else
            num = num + 1;
            laserMat( num, : ) = [ laserInput( i, j ) / 1000, rad( j - 2 ) ];
        end
    end
    
    if num == 0
        continue;
    end
    
    finalNum = 0;
    finalSet = zeros( num, 3 );

    curSet = 1;
    pointSet = zeros( num, 2 );
    pointSet( 1, : ) = [ 1, num ];
    
    while curSet ~= 0
                
        curPoint = pointSet( curSet, 1 );
        endPoint = pointSet( curSet, 2 );
        if curPoint == endPoint
            curSet = curSet - 1;
            continue;
        end
        
        up = laserMat( curPoint, 1 ) * cos( laserMat( curPoint, 2 ) ) - laserMat( endPoint, 1 ) * cos( laserMat( endPoint, 2 ) );
        down = laserMat( endPoint, 1 ) * sin( laserMat( endPoint, 2 ) ) - laserMat( curPoint, 1 ) * sin( laserMat( curPoint, 2 ) );
        
        a = atan2( up, down );
        r = cos( a ) * laserMat( curPoint, 1 ) * cos( laserMat( curPoint, 2 ) ) + sin( a ) * laserMat( curPoint, 1 ) * sin( laserMat( curPoint, 2 ) );
        
        maxDis = 0;
        maxPoint = 0;
        for j = curPoint + 1 : endPoint - 1
            if abs( laserMat( j, 1 ) * cos( laserMat( j, 2 ) - a ) - r ) >= maxDis
                maxDis = abs( laserMat( j, 1 ) * cos( laserMat( j, 2 ) - a ) - r );
                maxPoint = j;
            end
        end
        
        if maxDis <= threshold
            curSet = curSet - 1;
            finalNum = finalNum + 1;
            finalSet( finalNum, 1 ) = a;
            finalSet( finalNum, 2 ) = r;
            finalSet( finalNum, 3 ) = endPoint;
            continue;
        end
        
        
        pointSet( curSet + 1, 1 ) = curPoint;
        pointSet( curSet + 1, 2 ) = maxPoint;
        pointSet( curSet, 1 ) = maxPoint;
        pointSet( curSet, 2 ) = endPoint;
        
        curSet = curSet + 1;
        
    end
    
    before = 0;
    lineNum = 0;
    for j = 1 : finalNum

        if j > 1
            if before == 1 && mod( abs( finalSet( j, 1 ) - finalSet( j - 1, 1 ) ), pi ) < 0.2
                continue;
            end
            if finalSet( j, 3 ) - finalSet( j - 1, 3 ) < 30
                before = 0;
                continue;
            end
        end

        before = 1;

        lineNum = lineNum + 1;
        robotLine( lineNum, : ) = finalSet( j, 1 : 2 );
        
    end
    
    
    
    % EKF algorithm
    v = odomInput( i, 3 );
    w = odomInput( i, 4 );
    
    x = robotPos( 1 );
    y = robotPos( 2 );
    theta = robotPos( 3 );
    
    if i == 1
        T = 0;
    else
        T = ( odomInput( i, 2 ) - odomInput( i - 1, 2 ) ) / 1000;
    end
    
    %{
    if w > 0.1
        robotPos( 1 ) = x - v / w * sin( theta ) + v / w * sin( theta + w * T );
        robotPos( 2 ) = y + v / w * cos( theta ) - v / w * cos( theta + w * T );
        robotPos( 3 ) = theta + w * T;

        A = [ 1, 0, - v / w * cos( theta ) + v / w * cos( theta + w * T );
                   0, 1, - v / w * sin( theta ) + v / w * sin( theta + w * T );
                   0, 0, 1 ];
               
        W = [ cos( theta ) * T, 0; sin( theta ) * T, 0; 0, T ]; 

        robotCovar = A * robotCovar * A' + W * Q * W';
        
    else
       %}
        robotPos( 1 ) = x + v * cos( theta + w * T / 2 ) * T;
        robotPos( 2 ) = y + v * sin( theta + w * T / 2 ) * T;
        robotPos( 3 ) = theta + w * T;
        
        A = [ 1, 0, -v * sin( theta ) * T;
              0, 1, v * cos( theta ) * T;
              0, 0, 1 ];
        
        W = [ cos( theta + w * T / 2 ) * T, -v * sin( theta + w * T / 2 ) * T / 2; sin( theta + w * T / 2 ) * T, v * cos( theta + w * T / 2 ) * T / 2; 0, T ]; 
        
        robotCovar = A * robotCovar * A' + W * Q * W';
        
    %end
    
    i
    %robotPos
    %robotCovar
    mapLineUsed = zeros( mapLineLength );
    for j = 1 : lineNum
        
        
        x = robotPos( 1 );
        y = robotPos( 2 );
        theta = robotPos( 3 );
        
        alfa = robotLine( j, 1 );
        gama = robotLine( j, 2 );
        if gama < 0
            alfa = alfa + pi;
        end
        gama = abs( gama );
        
        minDis = assThreshold;
        minLine = 0;
        
        for k = 1 : mapLineLength
            
            alfa2 = mapLine( k, 1 ) - theta;
            gama2 = mapLine( k, 2 ) - x * cos( mapLine( k, 1 ) ) - y * sin( mapLine( k, 1 ) );
            
            if gama2 < 0
                alfa2 = alfa2 + pi;
            end
            gama2 = abs( gama2 );
            
            H = [ 0, 0, -1; -cos( alfa2 ), -sin( alfa2 ), 0 ];
            
            
            v = zeros( 2, 1 );
            sigma = zeros( 2, 2 );
            v( 1 ) = mod( alfa - alfa2, 2 * pi );
            if v( 1 ) >  pi
                v( 1 ) = v( 1 ) - 2 * pi;
            end
            v( 2 ) = gama - gama2;
            sigma = R;
            
            if det( sigma ) < 0.000001
                minLine = k;
                break;
            end
                
            dis = v' * ( eye( 2 ) / sigma ) *  v;
            
            if dis < assThreshold && dis < minDis
                if minDis ~= assThreshold
                    minDif = dis / minDis;
                end
                minDis = dis;
                minLine = k;
            end
            
        end
        
        if minLine == 0
            continue;
        end
        
        mapLineUsed( minLine ) = 1;

        alfa2 = mapLine( minLine, 1 ) - theta;
        gama2 = mapLine( minLine, 2 ) - x * cos( mapLine( minLine, 1 ) ) - y * sin( mapLine( minLine, 1 ) );

        if gama2 < 0
            alfa2 = alfa2 + pi;
        end
        gama2 = abs( gama2 );
        

        robotMea( 1 ) = mod( alfa - alfa2, 2 * pi );
        if robotMea( 1 ) >  pi
            robotMea( 1 ) = robotMea( 1 ) - 2 * pi;
        end
        robotMea( 2 ) = gama - gama2;
        
        H = [ 0, 0, -1; -cos( alfa2 ), -sin( alfa2 ), 0 ];

        if det( H * robotCovar * H' + R ) < 0.000001
            continue;
        end
            
        K = robotCovar * H' * inv( H * robotCovar * H' + R);
                
        robotPos = robotPos + K * robotMea;
        robotCovar = ( eye( 3 ) - K * H ) * robotCovar;
        
    end
    
    robotTra( i, 1 ) = robotPos( 1 );
    robotTra( i, 2 ) = robotPos( 2 );
    
    
    % draw ground truth

    for j = 1 : mapLineLength

        if abs( tan( mapLine( j, 1 ) ) ) < 1
            plot_y = -17:20:3;
            plot_x = ( mapLine( j, 2 ) - sin( mapLine( j, 1 ) ) * plot_y ) / cos( mapLine( j, 1 ) );
        else
            plot_x = -4:20:16;
            plot_y = ( mapLine( j, 2 ) - cos( mapLine( j, 1 ) ) * plot_x ) / sin( mapLine( j, 1 ) );
        end

        if mapLineUsed( j ) == 1
            plot( plot_x, plot_y, 'C--' );
        else
            plot( plot_x, plot_y, 'B--' );
        end
        hold on;
    end
    
    plot( mapPoint( :, 1 ), mapPoint( :, 2 ),  'K.');
    
    %draw robot
    plot( robotTra( 1 : i, 1 ), robotTra( 1 : i, 2 ),  'G:X');
    
    for j = 1 : lineNum

        a = robotLine( j, 1 ) + robotPos( 3 );
        r = robotLine( j, 2 ) + robotPos( 1 ) * cos( a ) + robotPos( 2 ) * sin( a );
        if abs( tan( a ) ) < 1
            plot_y = -17:20:3;
            plot_x = ( r - sin( a ) * plot_y ) / cos( a );
        else
            plot_x = -4:20:16;
            plot_y = ( r - cos( a ) * plot_x ) / sin( a );
        end

        plot( plot_x, plot_y, 'R--' );
        hold on;
    end
    
    for j = 1 : num

        plot_x = laserMat( j, 1 ) * cos( laserMat( j, 2 ) +  robotPos( 3 ) ) + robotPos( 1 );
        plot_y = laserMat( j, 1 ) * sin( laserMat( j, 2 ) +  robotPos( 3 ) ) + robotPos( 2 );
        
        plot( plot_x, plot_y, 'Y.' );
        hold on;
    end
    
    [ Vec, Val ] = eig( robotCovar( 1:2, 1:2 ) );
    
    Val2 = [ sqrt( Val( 1, 1 ) ), 0;
             0, sqrt( Val( 2, 2 ) ) ];
    
    Mats = Vec * Val2;
    
    t = 0:0.01:2*pi;
    x = cos(t);
    y = sin(t);
    plot( (Mats( 1, 1 ) * x + Mats( 1, 2 ) * y) * 3 + robotPos( 1 ), (Mats( 2, 1 ) * x + Mats( 2, 2 ) * y) * 3 + robotPos( 2 ),'m--');
    for j = 1 : 10
        
        plot( robotPos( 1 ) + j / 10 * cos( robotPos( 3 ) ), robotPos( 2 ) + j / 10 * sin( robotPos( 3 ) ),'m.');
    end
    
    
    axis equal;
    axis([-4, 16, -17, 3]);
    saveas(gcf, ['../picture/fig',sprintf('%.3d',i),'.png']);
    hold off;
    
    %if i == 28
    robotPos
    robotCovar
    %pause;
    %end
    
end





end

