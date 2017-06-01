function [ new_Y ] = resample_data( Y, X, new_X )

  [new_num, dim] = size(new_X);
  resampled_Y = zeros(new_num, dim);
  
  idx = 0
  for i=1:1:new_num:
    while X[idx] > new_X[idx]:
      idx = idx + 1
    end
      new_Y = Y[idx]
  end
  
  return new_Y  
end

