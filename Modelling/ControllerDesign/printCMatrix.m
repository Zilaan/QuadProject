function out = printCMatrix(A)


[m,n] = size(A);

s = sprintf('K = {');

for i = 1:m
    s = [s, '{'];
    for j = 1:n
        
        if j == 1
            s = [s, num2str(A(i,j))];
        else
            s = [s,', ', num2str(A(i,j))];
        end
        
    end
    if i == m
        s = [s, '}'];
    else
        s = [s, '},\n'];
    end
end
s = [s, '};'];

out = s;


end