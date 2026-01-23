function r_max = computeArmMaxReach(uvms)
    % uvms: instance of UvmsModel
    
    qmin = uvms.jlmin;
    qmax = uvms.jlmax;
    
    % Number of samples per joint (adjust for speed vs accuracy)
    nSamples = 5;
    
    % Generate a grid of joint configurations
    joint_samples = cell(7,1);
    for i = 1:7
        joint_samples{i} = linspace(qmin(i), qmax(i), nSamples);
    end
    
    % Initialize maximum radius
    r_max = 0;
    
    % Nested loops over all joint samples (brute-force)
    for q1 = joint_samples{1}
        for q2 = joint_samples{2}
            for q3 = joint_samples{3}
                for q4 = joint_samples{4}
                    for q5 = joint_samples{5}
                        for q6 = joint_samples{6}
                            for q7 = joint_samples{7}
                                q = [q1;q2;q3;q4;q5;q6;q7];
                                uvms.q = q;
                                uvms.updateTransformations();
                                p_tool = uvms.vTt(1:3,4);
                                r = norm(p_tool);
                                if r > r_max
                                    r_max = r;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
