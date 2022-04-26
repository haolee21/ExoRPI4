
function [row,col,val] = csc_matrix(mat)
col_ptr=1;
cur_col = 1;
row = [];
col = [];
val =[];
for i_c=1:size(mat,2)
    
    for i_r=1:size(mat,1)
        
        if(mat(i_r,i_c)~=0)
            val = [val,mat(i_r,i_c)];
            row = [row,i_r];
            
            if(cur_col==i_c)
                col = [col,col_ptr];
                cur_col = cur_col+1;
            end
            col_ptr = col_ptr+1;
        end
        
    end



end
col = [col,col_ptr]; %end of line
row = row-1;
col = col-1;



