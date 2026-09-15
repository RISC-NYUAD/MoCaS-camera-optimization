function [Cam ,rotationMatrix] = generateCameraForDirectionFromPosition(cameraPosition, direction)
    PROJECTION_MATRIX = ProjectionMatrix(deg2rad(75), 1, 0.1, 10000);
    IMAGE_SIZE = [300, 300];
    viewDir = direction / norm(direction);
    rotationMatrix = createRotationMatrix_new(viewDir);

    Cam = Camera(PROJECTION_MATRIX, IMAGE_SIZE, cameraPosition, rotationMatrix);
end

function rotationMatrix = createRotationMatrix_new(direction)
    % Create a rotation matrix to align the camera with the given direction
    up = [0, 0, 1]; % Default up direction
    if all(direction == [0, 0, 1]) || all(direction == [0, 0, -1])
        up = [0, 1, 0]; % Change up direction for Z-axis views
    end
    
    z = -direction / norm(direction); % Negative view direction (camera looks at origin)
    if (direction(2) < 0)
        x = cross(up, z); % Orthogonal vector
    else
        x = cross(z, up); % Orthogonal vector
    end
    x = x / norm(x);
    y = cross(z, x)/norm(cross(z, x));
    rotationMatrix = [x(:), y(:), z(:)];
end