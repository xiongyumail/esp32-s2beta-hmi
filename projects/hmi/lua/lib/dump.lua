local dump = { _version = "0.1.0" }

---
-- @function: 获取table的字符串格式内容，递归
-- @tab： table
-- @ind：不用传此参数，递归用（前缀格式（空格））
-- @return: format string of the table
function dump.table(tab,ind)
    if(tab==nil)then return "nil" end;
    local str="{";
    if(ind==nil)then ind="  "; end;
    --//each of table
    for k,v in pairs(tab) do
      --//key
      if(type(k)=="string")then
        k=tostring(k).." = ";
      else
        k="["..tostring(k).."] = ";
      end;--//end if
      --//value
      local s="";
      if(type(v)=="nil")then
        s="nil";
      elseif(type(v)=="boolean")then
        if(v) then s="true"; else s="false"; end;
      elseif(type(v)=="number")then
        s=v;
      elseif(type(v)=="string")then
        s="\""..v.."\"";
      elseif(type(v)=="table")then
        s=dump.table(v,ind.."  ");
        s=string.sub(s,1,#s-1);
      elseif(type(v)=="function")then
        s="function : "..v;
      elseif(type(v)=="thread")then
        s="thread : "..tostring(v);
      elseif(type(v)=="userdata")then
        s="userdata : "..tostring(v);
      else
        s="nuknow : "..tostring(v);
      end;--//end if
      --//Contact
      str=str.."\n"..ind..k..s.." ,";
    end --//end for
    --//return the format string
    local sss=string.sub(str,1,#str-1);
    if(#ind>0)then ind=string.sub(ind,1,#ind-2) end;
    sss=sss.."\n"..ind.."}\n";
    return sss;--string.sub(str,1,#str-1).."\n"..ind.."}\n";
  end;--//end function

function dump.uri(uri)
    local t = {}
    for key, value in string.gmatch(uri, '(%w+)=(%w+)') do
        t[key] = value
    end
    return t
end

return dump