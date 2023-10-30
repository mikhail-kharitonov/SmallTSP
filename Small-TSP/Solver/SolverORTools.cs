using Google.OrTools;
using Google.OrTools.ConstraintSolver;
using Small_TSP.DataModel;
using Google.Protobuf.WellKnownTypes;
namespace Small_TSP.Solver;

public class SolverORTools
{
    private static int _amountVehicles = 1;
    private static int _timeLimitForSolutionSeconds = 1;
    
    private int GetDist(GeoPoint pointFrom, GeoPoint pointTo, List<ArcImprovedRoute> arcs)
    {
        int result = 0;
        IEnumerable<int> distance = arcs
            .Where(a => a.PointFrom.Equals(pointFrom) 
                        && a.PointTo.Equals(pointTo))
            .Select(d => d.Distance);
        
        foreach (int d in distance)
        {
            result =  d;
        }
        return result;
    }

    private int[,] BuldDistance(Dictionary<int, GeoPoint> arcsFrom, Dictionary<int, GeoPoint> arcsTo, List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        int[,] distance = new int[arcsFrom.Count, arcsTo.Count];
        
        foreach (int row in arcsFrom.Keys)
        {
            foreach (int column in arcsTo.Keys)
            {
                distance[row, column] = GetDist(arcsFrom[row], arcsTo[column], arcsImprovedRoutes);
            }
        }
        return distance;
    }

    public (Dictionary<int, GeoPoint> arcsFrom, Dictionary<int, GeoPoint> arcsTo)  CreateNumbersPoints(List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        Dictionary<int, GeoPoint> arcsFrom = new Dictionary<int, GeoPoint>();
        Dictionary<int, GeoPoint> arcsTo = new Dictionary<int, GeoPoint>();
        
        int row = 0;
        int column = 0;

        foreach (ArcImprovedRoute arc in arcsImprovedRoutes)
        {
            if (!arcsFrom.ContainsValue(arc.PointFrom))
            {
                arcsFrom.Add(row, arc.PointFrom);
                row++;
            }
            
            if (!arcsTo.ContainsValue(arc.PointTo))
            {
                arcsTo.Add(column, arc.PointTo);
                column++;
            }
        }
        return (arcsFrom, arcsTo);
    }

    private Assignment Solve(RoutingIndexManager manager, RoutingModel routing, int[,] distanceMatrix)
    {
        int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
        {
            int fromNode = manager.IndexToNode(fromIndex);
            int toNode = manager.IndexToNode(toIndex);
            return distanceMatrix[fromNode, toNode];
        });
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

        RoutingSearchParameters searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
        searchParameters.TimeLimit = new Duration { Seconds = _timeLimitForSolutionSeconds };

        return routing.SolveWithParameters(searchParameters);
    }

    public List<int> GetMaskRoutePoints(List<ArcImprovedRoute> arcsImprovedRoutes, GeoPoint pointStart, GeoPoint pointEnd)
    {
        (Dictionary<int, GeoPoint> arcsFrom, Dictionary<int, GeoPoint> arcsTo) = CreateNumbersPoints(arcsImprovedRoutes);

        int[,] distance = BuldDistance(arcsFrom, arcsTo, arcsImprovedRoutes);
        int startPoint = arcsFrom.FirstOrDefault(p => p.Value.Equals(pointStart)).Key;
        int endPoint = arcsFrom.FirstOrDefault(p => p.Value.Equals(pointEnd)).Key;
        int amountNodes = distance.GetLength(0);
        int[] starts = new int [1] { startPoint };
        int[] ends = new int [1] { endPoint };
        
        RoutingIndexManager manager = new RoutingIndexManager(amountNodes, _amountVehicles, starts, ends);
        RoutingModel routing = new RoutingModel(manager);
        Assignment solution = Solve(manager, routing, distance);
        List<int> routePoints = GetRouteNumberPoints(routing, manager, solution, endPoint);
        return routePoints;
    }
    
    private List<int> GetRouteNumberPoints(RoutingModel routing, RoutingIndexManager manager, Assignment solution, int endPoint)
    {
        List<int> routeNumberPoints = new List<int>();
        long index = routing.Start(0);
        while (!routing.IsEnd(index))
        {
            routeNumberPoints.Add(manager.IndexToNode((int)index));
            index = solution.Value(routing.NextVar(index));
        }
        routeNumberPoints.Add(endPoint);
        return routeNumberPoints;
    }

}