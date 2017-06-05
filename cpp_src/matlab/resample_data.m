function [ new_Y ] = resample_data( Y, X, new_X )

  [new_numX, dim] = size(new_X);
  [numY,dim] = size(Y);
  new_Y = zeros(new_numX,dim);
  
  idx = 1
  for i=1:1:new_numX
    if idx == 1 && X(idx,1) > new_X(i,1)
       new_Y(i,1) = inf;
       continue;
    end
    while X(idx,1) < new_X(i,1) && idx < numY  
         idx = idx + 1;
    end
    new_Y(i,1) = Y(idx,1);
  end

end

