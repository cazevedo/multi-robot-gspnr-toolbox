function list_of_names = translate_to_names(element_list, indices_list)
    
    list_of_names = [string.empty];
    
    nIndices = size(indices_list, 2);
    
    for i_index = 1:nIndices
        index = indices_list(i_index);
        name = element_list(index);
        list_of_names = cat(2, list_of_names, name);
    end
end