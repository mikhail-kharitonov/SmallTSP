namespace Small_TSP.DataProcessors.Interfaces;

public interface IFileManager
{
    public string Read(string fileName);

    public void Write(string fileName, string fileData);

    public void Delete(string fileName);
}