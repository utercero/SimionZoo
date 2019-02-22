/*
	SimionZoo: A framework for online model-free Reinforcement Learning on continuous
	control problems

	Copyright (c) 2016 SimionSoft. https://github.com/simionsoft

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

using Caliburn.Micro;
using System.Collections.Generic;
using System.Runtime.Serialization;

using Badger.Data;

namespace Badger.ViewModels
{
    [DataContract]
    public class ReportViewModel : PropertyChangedBase
    {
        private string m_name = "Unnamed";
        [DataMember]
        public string Name { get { return m_name; } set { m_name = value; NotifyOfPropertyChange(() => Name); } }

        //plots
        private PlotViewModel m_plot= null;
        [DataMember]
        public PlotViewModel Plot
        {
            get { return m_plot; }
            set { m_plot = value; m_plot.UpdateView(); NotifyOfPropertyChange(() => Plot); }
        }
 
        //stats
        private StatsViewModel m_stats;
        [DataMember]
        public StatsViewModel Stats
        {
            get { return m_stats; }
            set { m_stats = value; NotifyOfPropertyChange(() => Stats); }
        }

        public void ExportNonSerializable(string outputFolder, ref Dictionary<string, string> outputFiles)
        {
            //export plots
            if (Plot!=null) Plot.Export(outputFolder, ref outputFiles);
        }

        public void ImportNonSerializable(string inputFolder)
        {
            if (Plot != null)
            {
                Plot.IsNotifying = true;
                Plot.Import(inputFolder);
            }
        }

        private ReportType GetVariableReportType(LogQueryViewModel query, string variable)
        {
            //TODO: this only returns the first report type for that variable
            //This should be improved
            foreach (LoggedVariableViewModel var in query.VariablesVM)
            {
                if (var.Name == variable)
                    return var.SelectedPlotType;
            }
            return ReportType.Undefined;
        }

        public ReportViewModel(List<TrackGroup> queryResultTracks, LogQueryViewModel query, Report report)
        {
            Name = report.Name;

            //Create the plot
            PlotViewModel newPlot = new PlotViewModel(report.Name, "Time (s)", report.Name, false, true);
            //Create the stats
            StatsViewModel newStatGroup = new StatsViewModel(report.Name);

            foreach (TrackGroup group in queryResultTracks)
            {
                //plot data
                if (group.ConsolidatedTrack != null)
                {
                    SeriesGroup seriesGroup = group.ConsolidatedTrack.SeriesGroups[report];

                    foreach (Series series in seriesGroup.SeriesList)
                    {
                        string seriesName;
                        string description;
                        if (seriesGroup.SeriesList.Count == 1)
                        {
                            //only one series per track group, no multi-series track group
                            seriesName = group.ConsolidatedTrack.TrackId;
                            description = group.ConsolidatedTrack.FullTrackId;
                        }
                        else
                        {
                            seriesName = group.ConsolidatedTrack.TrackId + "-" + series.Id;
                            description = group.ConsolidatedTrack.FullTrackId + "-" + series.Id;
                        }

                        //add data to the plot
                        int lineSeriesId = newPlot.AddLineSeries(seriesName, description);
                        foreach (XYValue value in series.Values)
                            newPlot.AddLineSeriesValue(lineSeriesId, value.X, value.Y);

                        StatViewModel newStat =
                            new StatViewModel(group.ExperimentId, seriesName, series.Stats
                                , group.ConsolidatedTrack.LogBinaryFile
                                , group.ConsolidatedTrack.LogDescriptorFile
                                , group.ConsolidatedTrack.ExperimentalUnitConfigFile);

                        newStatGroup.addStat(newStat);
                    }
                }
            }
            Plot = newPlot;
            Stats = newStatGroup;
        }

    }
}
