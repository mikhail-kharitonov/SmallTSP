using Small_TSP.DataProcessors.Interfaces;

namespace Small_TSP.DataProcessors;

public class FileManager: IFileManager
{
    public string Read(string fileName)
    {
        return File.ReadAllText(fileName);
    }

    public void Write(string fileName, string fileData) 
    {
        File.WriteAllText(fileName, fileData);
    }

    public void Delete(string fileName)
    {
        File.Delete(fileName);
    }
}