namespace Small_TSP.DataModel;

public readonly record struct ArcImprovedRoute (GeoPoint PointFrom, 
    GeoPoint PointTo, 
    int Distance, 
    int Duration);
