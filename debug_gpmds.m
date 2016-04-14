function debug_gpmds(fig,ds_handle)

figure(fig);
while(1)
x = get_point(fig);
feval(ds_handle,x)
end

end