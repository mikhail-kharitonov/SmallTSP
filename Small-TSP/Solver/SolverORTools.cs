using Google.OrTools;
using Google.OrTools.ConstraintSolver;
using Small_TSP.DataModel;
using Google.Protobuf.WellKnownTypes;
namespace Small_TSP.Solver;

public class SolverORTools
{
    private static int _amountVehicles = 1;
    private static int _timeLimitForSolutionSeconds = 1;
    
    private int GetDist(string arcFrom, string arcTo, List<ArcImprovedRoute> arcs)
    {
        int result = 0;
        IEnumerable<int> distance = arcs
            .Where(a => a.ArcFrom.Equals(arcFrom) && a.ArcTo.Equals(arcTo))
            .Select(d => d.Distance);
        
        foreach (int d in distance)
        {
            result =  d;
        }
        return result;
    }

    private int[,] BuldDistance(Dictionary<string, int> arcsFrom, Dictionary<string, int> arcsTo, List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        int[,] distance = new int[arcsFrom.Count, arcsTo.Count];
        int row = 0;
        int column = 0;
        
        foreach (string arcFrom in arcsFrom.Keys)
        {
            foreach (string arcTo in arcsTo.Keys)
            {
                distance[row, column] = GetDist(arcFrom, arcTo, arcsImprovedRoutes);
                column++;
            }
            row++;
            column = 0;
        }
        return distance;
    }

    private (Dictionary<string, int> arcsFrom, Dictionary<string, int> arcsTo)  CreateNumbersPoints(List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        Dictionary<string, int> arcsFrom = new Dictionary<string, int>();
        Dictionary<string, int> arcsTo = new Dictionary<string, int>();
        
        int row = 0;
        int column = 0;

        foreach (ArcImprovedRoute arc in arcsImprovedRoutes)
        {
            if (!arcsFrom.ContainsKey(arc.ArcFrom))
            {
                arcsFrom.Add(arc.ArcFrom, row);
                row++;
            }

            if (!arcsTo.ContainsKey(arc.ArcTo))
            {
                arcsTo.Add(arc.ArcTo, column);
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

    public List<int> GetMaskRoutePoints(List<ArcImprovedRoute> arcsImprovedRoutes, string maskStart, string maskEnd)
    {
        (Dictionary<string, int> arcsFrom, Dictionary<string, int> arcsTo) = CreateNumbersPoints(arcsImprovedRoutes);

        int[,] distance = BuldDistance(arcsFrom, arcsTo, arcsImprovedRoutes);
        int startPoint = arcsFrom[maskStart];
        int endPoint = arcsTo[maskEnd];
        int amountNodes = distance.GetLength(0);
        int[] starts = new int [1] { startPoint };
        int[] ends = new int [1] { endPoint };
        
        RoutingIndexManager manager = new RoutingIndexManager(amountNodes, _amountVehicles, starts, ends);
        RoutingModel routing = new RoutingModel(manager);
        Assignment solution = Solve(manager, routing, distance);
        List<int> routePoints = GetRouteNumberPoints(routing, manager, solution, endPoint);
        List<string> maskRoutePoints = new List<string>();

        foreach (int point in routePoints)
        {
            string mask = arcsFrom.FirstOrDefault(n => n.Value == point).Key;
            maskRoutePoints.Add(mask);
        }
        return routePoints;
    }
    
    private List<int> GetRouteNumberPoints(RoutingModel routing, RoutingIndexManager manager, Assignment solution, int endPoint)
    {
        List<int> routeNumberPoints = new List<int>();
        long index = routing.Start(0);
        while (routing.IsEnd(index) == false)
        {
            routeNumberPoints.Add(manager.IndexToNode((int)index));
            index = solution.Value(routing.NextVar(index));
        }
        routeNumberPoints.Add(endPoint);
        return routeNumberPoints;
    }
    
}