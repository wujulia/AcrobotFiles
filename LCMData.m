classdef LCMData < handle
  %LCMDATA 
  
  properties
    t = [];
    data = [];
    index = 0;
  end
  
  methods
    function obj = LCMData()
    end
    
    function obj=addData(obj,t,x)
      if isempty(obj.t)
        obj.t = zeros(length(t),1e4);
        obj.data = zeros(length(x),1e4);
      end
      obj.index = obj.index+1;
      obj.t(:,obj.index) = t;
      obj.data(:,obj.index) = x;
      
      if obj.index == size(obj.t,2),
        obj.data = [obj.data zeros(size(obj.data,1),size(obj.t,2))];
        obj.t = [obj.t zeros(size(obj.t,1),size(obj.t,2))];
      end
    end

    function obj = truncateData(obj)
      obj.t = obj.t(:,1:obj.index);
      obj.data = obj.data(:,1:obj.index);
    end
  end
 
end

