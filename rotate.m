function rotate(name, transform)

n = name;
t = transform;

vertices = get(n,'Vertices'); %getting the vertices from the ply model
%rotating person 180 degrees (pi)
transformedVertices = [vertices,ones(size(vertices,1),1)] * t';
set(n,'Vertices',transformedVertices(:,1:3));

end