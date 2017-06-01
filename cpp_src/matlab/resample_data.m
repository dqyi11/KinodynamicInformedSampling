function [ resampled_data ] = resample_data( data, total_time, interval )

  [num, dim] = size(data);
  new_num = int(total_time/interval);
  resampled_data = zeros(new_num, dim);
  
  idx = 1
  for 
  
end

