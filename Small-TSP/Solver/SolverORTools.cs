using Google.OrTools;
using Google.OrTools.ConstraintSolver;
using Small_TSP.DataModel;
using Google.Protobuf.WellKnownTypes;
namespace Small_TSP.Solver;

public class SolverORTools
{
    private static int _vehicleNumber = 1;
    private static int _timeLimitSeconds = 1;
    
    private int GetDist(string arcFrom, string arcTo, List<ArcImprovedRoute> distanceMatrix)
    {
        int result = 0;
        IEnumerable<int> distance = distanceMatrix
            .Where(a => a.ArcFrom.Equals(arcFrom) && a.ArcTo.Equals(arcTo))
            .Select(d => d.Distance);
        
        foreach (int d in distance)
        {
            result =  d;
        }

        return result;
    }
    

    public int[,] BuildDistanceMatrix(List<ArcImprovedRoute> distanceMatrix)
    {
        Dictionary<string, int> arcsFrom = new Dictionary<string, int>();
        Dictionary<string, int> arcsTo = new Dictionary<string, int>();
        
        int row = 0;
        int column = 0;

        foreach (ArcImprovedRoute arc in distanceMatrix)
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
        int[,] distance = new int[arcsFrom.Count, arcsTo.Count];
        row = 0;
        column = 0;
        
        foreach (string arcFrom in arcsFrom.Keys)
        {
            foreach (string arcTo in arcsTo.Keys)
            {
                distance[row, column] = GetDist(arcFrom, arcTo, distanceMatrix);
                column++;
            }
            row++;
            column = 0;
        }

        return distance;
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
        searchParameters.TimeLimit = new Duration { Seconds = _timeLimitSeconds };

        return routing.SolveWithParameters(searchParameters);
    }

    private int[,] BuildVariables(RoutingModel routing, Assignment solution, int[,] distanceMatrix)
    {
        long index = routing.Start(0);
        int amount = distanceMatrix.GetLength(0);
        int [,] variables = new int[amount, amount];
        long previousIndex;

        while (routing.IsEnd(index) == false)
        {
            previousIndex = index;
            index = solution.Value(routing.NextVar(index));
            if (index <= amount-1)
            {
                variables[previousIndex, index] = 1;
            }
            else
            {
                variables[previousIndex, routing.Start(0)] = 1;
            }
        }
        return variables;
    }

    public (int[,], long) GetSolution(List<ArcImprovedRoute> distanceMatrix, int startPoint, int endPoint)
    {
        int[,] distance = BuildDistanceMatrix(distanceMatrix);
        int count = distance.GetLength(0);
        int[] starts = new int [1] { startPoint };
        int[] ends = new int [1] { endPoint };
        
        RoutingIndexManager manager = new RoutingIndexManager(count, _vehicleNumber, starts, ends);
        RoutingModel routing = new RoutingModel(manager);
        Assignment solution = Solve(manager, routing, distance);
        int[,] variables = BuildVariables(routing, solution, distance);
        long objective = solution.ObjectiveValue();
        return (variables, objective);
    }

}