function [ result ] = isIn( current, ending, range )
    if (abs(current - ending) < range )
        result = true;
    else
        result = false;
    end
end

