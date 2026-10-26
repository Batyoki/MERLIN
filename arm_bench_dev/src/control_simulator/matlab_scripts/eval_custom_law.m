function u = eval_custom_law(law_text, param_names, param_values, e, dt, integral, derivative, x, r, t)
% EVAL_CUSTOM_LAW Evaluate a free-text control law in MATLAB.
% The law should assign to 'u' and may reference e, dt, integral,
% derivative, x, r, t, plus any user-defined parameter names.

    for i = 1:length(param_names)
        param_name = param_names{i};
        param_value = param_values(i);
        eval([param_name ' = ' num2str(param_value) ';']);
    end

    eval(law_text);
end
