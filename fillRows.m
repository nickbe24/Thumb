function new_m = fillRows(m)

    last_val = find(any(m, 2), 1, 'last');
    last_val_Row = m(last_val, :);
    for i = last_val+1:size(m, 1)
        m(i, :) = last_val_Row;
    end
    new_m = m;
end
