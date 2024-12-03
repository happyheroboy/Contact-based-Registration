function [tri, fileform, A, S] = stlread(filename)
%STLREAD Create triangulation from STL file
%   TR = STLREAD(filename) returns a triangulation object TR containing the
%   triangles defined in an STL file.
%
%   [TR,fileformat] = STLREAD(filename) additionally
%   returns the format of the file, either 'binary' or 'text'.
%
%   [TR,fileformat,attributes] = STLREAD(filename) additionally returns
%   binary attributes, represented as a uint16 vector of length equal to
%   the number of triangles. This is only supported for binary files - if
%   the input file is in text format, attributes is empty.
%
%   [TR,fileformat,attributes,solidID] = STLREAD(filename) additionally
%   returns a vector of identification numbers for text files. The
%   identification numbers assign each triangle to a grouping of triangles
%   in the triangulation. If the input file is binary, solidID contains all
%   ones.

% Copyright 2018 The MathWorks, Inc.

if ~isScalarText(filename)
    error(message("MATLAB:polyfun:stlFileName"));
end

% Get absolute path to the file (accounting for MATLAB path etc)
try
    filenameCell = matlab.io.internal.validators.validateFileName(filename);
catch
    filenameCell = {filename}; % If we can't open, throw the error below.
end

ST = matlab.internal.meshio.stlread(filenameCell{1});

switch ST.ErrorCode
    case 0 % NO_ERROR
    case 1 % INVALID_FILE_EXTENSION
        error(message("MATLAB:polyfun:stlFileExtension"));
    case 2 % assume 2 - FILE_OPEN_FAILED
        error(message("MATLAB:polyfun:stlFileCannotOpen", filename));
    otherwise
        % One of these error codes:
        % 3 - FILE_EMPTY (file is empty)
        % 4 - INVALID_STL_FORMAT (file has invalid format)
        % 5 - FILE_WRITE_FAILED (stlwrite only)
        % 6 - INVALID_DATA (stlwrite only)
        error(message("MATLAB:polyfun:stlFailedToRead", filename));
end

if size(ST.Vertices, 1) < 3
    %stl file contains degenerate data or overflow/underflow
    error(message("MATLAB:polyfun:stlInvalidData"));
end

tri = triangulation(ST.Faces, ST.Vertices);

%check and flip normals
normals = faceNormal(tri);
product = dot(normals, ST.Normals, 2);

% Note: This makes triangle orientation consistent with the normals given
% in the STL file. It does not check that adjacent triangles have
% consistent orientation.
tf = (product < -0.1);
if any(tf)
    Facets = ST.Faces;
    Facets(tf, [2 3]) = Facets(tf, [3 2]);
    tri = triangulation(Facets, ST.Vertices);
end

S = ST.SolidIndex;
A = ST.Attributes;
fileform = ST.SourceFormat;
end


function tf = isScalarText(c)
    tf = (ischar(c) && (isrow(c) || isequal(size(c), [0 0]))) || (isstring(c) && isscalar(c));
end