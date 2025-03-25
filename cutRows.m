function newm = cutRows(m)

    last_val = find(any(m, 2), 1, 'last');
    newm = m(1:last_val,:);

end
