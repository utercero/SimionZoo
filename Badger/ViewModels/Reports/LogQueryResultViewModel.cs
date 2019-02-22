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

using System.Collections.Generic;
using System.Runtime.Serialization;

using Caliburn.Micro;

using Badger.Data;


namespace Badger.ViewModels
{
    [DataContract]
    public class LogQueryResultViewModel : PropertyChangedBase
    {
        static int numQueryResults = 0;

        string getDefaultQueryResultName()
        {
            numQueryResults++;
            return "Query-" + numQueryResults;
        }

        [DataMember]
        public LogQueryViewModel Query { get; set; }

        string m_name = "Unnamed";
        [DataMember]
        public string Name
        {
            get { return m_name; }
            set { m_name = value; NotifyOfPropertyChange(() => Name); }
        }

        //report list
        [DataMember]
        public BindableCollection<ReportViewModel> Reports
        { get; set; } = new BindableCollection<ReportViewModel>();

        //report selection in tab control
        private ReportViewModel m_selectedReport;
        public ReportViewModel SelectedReport
        {
            get { return m_selectedReport; }
            set
            {
                m_selectedReport = value;
                NotifyOfPropertyChange(() => SelectedReport);
            }
        }

        public LogQueryResultViewModel(List<TrackGroup> queryResultTracks, List<Report> reports, LogQueryViewModel query)
        {
            Name = getDefaultQueryResultName();

            Query = query;

            // Display the reports
            foreach (Report report in reports)
            {
                ReportViewModel newReport = new ReportViewModel(queryResultTracks, query, report);
                Reports.Add(newReport);
            }

            //Set the last as selected
            if (Reports.Count > 0)
                SelectedReport = Reports[Reports.Count-1];
        }

        public Dictionary<string,string> ExportNonSerializable(string outputBaseFolder)
        {
            Dictionary<string, string> outputFiles = new Dictionary<string, string>();
            foreach (ReportViewModel report in Reports)
            {
                report.ExportNonSerializable(outputBaseFolder, ref outputFiles);
            }
            return outputFiles;
        }

        public void ImportNonSerializable(string inputBaseFolder)
        {
            foreach (ReportViewModel report in Reports)
            {
                report.ImportNonSerializable(inputBaseFolder);
            }
            if (Reports.Count > 0) SelectedReport = Reports[0];
        }


        public void SetNotifying(bool notifying)
        {
            IsNotifying = notifying;
            foreach (ReportViewModel report in Reports)
                report.IsNotifying = notifying;
        }
    }
}
