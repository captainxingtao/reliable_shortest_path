//  Copyright 2008-2012 Tao Xing, Xuesong Zhou @ University of Utah
//  tao.xing@utah.edu
//  This file is part of Shortest path engine (SP_Engine)

//  SP_Engine is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  any later version.

//  SP_Engine is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

//  You should have received a copy of the GNU Lesser General Public License
//  along with SP_Engine.  If not, see <http://www.gnu.org/licenses/>.

using System;
using System.Data;
using System.Configuration;
using System.Text;

using System.Drawing;
using System.Drawing.Imaging;

using System.IO;
using System.IO.Compression;
using System.Collections;
using System.Collections.Generic;

using System.Xml;
using System.Net;

using UTMconv;

//using MySql.Data.MySqlClient;

namespace Routing_Engine
{




    //Node class
    public class CNode
    {
        public int Node_ID, Node_AreaID;
        public double Node_Long, Node_Lat, Node_UTM_Easting, Node_UTM_Northing;
        public int Node_UTM_Zone, Node_MapLevel, Node_ControlType, Node_Type;
        public string Node_Name;

        //constructor
        public CNode()
        {
            Node_ID = 0;
            Node_AreaID = 0;
            Node_Long = 0;
            Node_Lat = 0;
            Node_UTM_Easting = 0;
            Node_UTM_Northing = 0;
            Node_UTM_Zone = 0;
            Node_MapLevel = 0;
            Node_ControlType = 0;
            Node_Type = 0;
            Node_Name = null;
        }
        //overloading constructor
        public CNode(int _Node_ID, int _Node_AreaID, double _Node_Long, double _Node_Lat, double _Node_UTM_Easting, double _Node_UTM_Northing, int _Node_UTM_Zone,
            int _Node_MapLevel, int _Node_ControlType, int _Node_Type, string _Node_Name)
        {
            Node_ID = _Node_ID;
            Node_AreaID = _Node_AreaID;
            Node_Long = _Node_Long;
            Node_Lat = _Node_Lat;
            Node_UTM_Easting = _Node_UTM_Easting;
            Node_UTM_Northing = _Node_UTM_Northing;
            Node_UTM_Zone = _Node_UTM_Zone;
            Node_MapLevel = _Node_MapLevel;
            Node_ControlType = _Node_ControlType;
            Node_Type = _Node_Type;
            Node_Name = _Node_Name;
        }

    }

    //Link class
    public class CLink
    {
        public DirectionalID ID;
        public int AreaID, FromID, ToID, MapLevel, ModeFlag, LaneSize, Link_Type, ShapeID, Capacity, SensorID;
        public double Link_Length, SpeedLimit, TravelTime, Safety, TTVariance;//unit
        public string Link_Name, TMC;
        private double Feet2Mile = 0.1894F; //1 foot = 0.0001894 mile
        public bool ShapeDir, IsShape, RealData; //direction of shape point stored in the aryCShape
        // Link_Type == 2: free way; Link_Type > 2 arterial

        //constructor
        public CLink()
        {
            ID = new DirectionalID();//
            AreaID = 0;//
            Link_Name = null;
            FromID = 0;//
            ToID = 0;//
            MapLevel = 0;//
            ModeFlag = 0;//
            Capacity = 0;
            Link_Length = 0;// mile
            LaneSize = 0;//
            SpeedLimit = 0;// mph
            TravelTime = 0;// min
            Link_Type = 0;//
            IsShape = false;
            RealData = false;
            ShapeID = 0;
            ShapeDir = true;
            TMC = null;
            SensorID = 0;
            Safety = 0;
            TTVariance = 0;
        }

        //overloading constructor
        public CLink(DirectionalID _Link_ID, int _Link_AreaID, string _Link_Name, int _Link_FromID, int _Link_ToID, int _Link_MapLevel, int _Link_ModeFlag,
                     string _Link_Length_Unit, int _Link_Capacity, double _Link_Length, int _Link_LaneSize, double _Link_SpeedLimit, int _Link_Type, bool _Link_IsShape,
                     int _Link_ShapeID, bool _Link_ShapeDir, double _Link_Safety, bool _Link_RealData, int _SensorID)
        {
            ID = _Link_ID;
            AreaID = _Link_AreaID;
            Link_Name = _Link_Name;
            FromID = _Link_FromID;
            ToID = _Link_ToID;
            MapLevel = _Link_MapLevel;
            ModeFlag = _Link_ModeFlag;
            Capacity = _Link_Capacity;
            if (_Link_Length_Unit == "feet")
            {
                Link_Length = _Link_Length / 1000 * Feet2Mile;
            }
            if (_Link_Length_Unit == "mile")
            {
                Link_Length = _Link_Length;
            }
            LaneSize = _Link_LaneSize;
            SpeedLimit = _Link_SpeedLimit;//unit: mph
            TravelTime = Link_Length / SpeedLimit * 60;
            Link_Type = _Link_Type;
            IsShape = _Link_IsShape;
            ShapeID = _Link_ShapeID;
            ShapeDir = _Link_ShapeDir;
            TMC = null;
            SensorID = _SensorID;
            Safety = _Link_Safety;
            TTVariance = 0;
            RealData = _Link_RealData;
        }

    }

    public class CSensor
    {
        public int sensorID;
        public int sensor_type;
        // sensor type: 0: loop detector, 1: AVI, 2: probe(GPS)
        /// <summary>
        /// constructor
        /// </summary>
        public CSensor()
        {
            sensorID = 0;
            sensor_type = 0;
        }

        /// <summary>
        /// Overloading constructor
        /// </summary>
        /// <param name="_ID">ID</param>
        /// <param name="_Dir">Direction</param>
        public CSensor(int _sensorID, int _sensor_type)
        {
            sensorID = _sensorID;
            sensor_type = _sensor_type;
        }
    }

    public class DirectionalID
    {
        public long ID;
        public bool Dir;

        /// <summary>
        /// constructor
        /// </summary>
        public DirectionalID()
        {
            ID = 0;
            Dir = true;
        }

        /// <summary>
        /// Overloading constructor
        /// </summary>
        /// <param name="_ID">ID</param>
        /// <param name="_Dir">Direction</param>
        public DirectionalID(long _ID, bool _Dir)
        {
            ID = _ID;
            Dir = _Dir;
        }

        public override bool Equals(object obj)
        {
            if (!(obj is DirectionalID))
                return false;
            DirectionalID other = (DirectionalID)obj;
            return other.ID == ID && other.Dir == Dir;
        }

        public bool Equals(DirectionalID id)
        {
            return id.ID == ID && id.Dir == Dir;
        }

        public override int GetHashCode()
        {
            return (int)(ID + ((Dir) ? 1 : 0));
        }
    }

    public class CRoute
    {
        public int[] PathAry; //array contains a list of nodes in index
        public int[] LinkSeq; //array contains a list of links in index
        public int[] TrafficColor;
        public double PathTime, PathDist, PathSafety, PathCost;

        /// <summary>
        /// constructor
        /// </summary>
        public CRoute()
        {
            PathAry = null;
            LinkSeq = null;
            TrafficColor = null;
            PathTime = 0;
            PathDist = 0;
            PathSafety = 1;
            PathCost = 0;
        }

        /// <summary>
        /// Overloading constructor
        /// </summary>
        /// <param name="_PathAry">array contains a list of nodes in index</param>
        /// <param name="_LinkSeq">array contains a list of links in index</param>
        /// <param name="_TrafficColor">array contains a list of traffic color code</param>
        /// <param name="_PathTime">travel time in minute</param>
        /// <param name="_PathDist">distance in mile</param>
        /// <param name="_PathSafety">safety factor in percentage</param>
        /// <param name="_PathCost">cost in dollar</param>
        public CRoute(int[] _PathAry, int[] _LinkSeq, int[] _TrafficColor, double _PathTime, double _PathDist, double _PathSafety, double _PathCost)
        {
            PathAry = _PathAry;
            LinkSeq = _LinkSeq;
            TrafficColor = _TrafficColor;
            PathTime = _PathTime;
            PathDist = _PathDist;
            PathSafety = _PathSafety;
            PathCost = _PathCost;
        }

        //TODO: override function equals
    }

    public class CBound
    {
        public double up, low, left, right;
        public CBound()
        {
            up = 0;
            low = 0;
            left = 0;
            right = 0;
        }
        public CBound(double _up, double _low, double _left, double _right)
        {
            up = _up;
            low = _low;
            left = _left;
            right = _right;
        }
    }

    //Shape class
    public class CShape
    {
        public int Shape_ID, Shape_Length;
        public double[] Shape_Long, Shape_Lat;

        //constructor
        public CShape()
        {
            Shape_ID = 0;
            Shape_Length = 0;
            Shape_Lat = null;
            Shape_Long = null;
        }

        //overloading constructor
        public CShape(int _Shape_ID, int _Shape_Length, double[] _Shape_Lat, double[] _Shape_Long)
        {
            Shape_ID = _Shape_ID;
            Shape_Length = _Shape_Length;
            Shape_Lat = _Shape_Lat;
            Shape_Long = _Shape_Long;
        }

    }

    //TODO: add CRoutingRequest class
    //TODO: add Traffic class

    public class CgpsTrace
    {
        public string MetaUserId;
        public int MetaTraceId;
        public DateTime MetaInsertDate;
        public double Lat, Long, Speed, Heading, RelativeTime, SeaLevelAlt;
        public double east, north;
        public int utm_zone, NumOfSat;
        public string GpsTime;

        public CgpsTrace()
        {
            MetaUserId = "";
            MetaTraceId = 0;
            MetaInsertDate = new DateTime();
            Lat = 0;
            Long = 0;
            east = 0;
            north = 0;
            utm_zone = 0;
            Speed = 0;
            Heading = 0;
            RelativeTime = 0;
            SeaLevelAlt = 0;
            NumOfSat = 0;
            GpsTime = "";
        }

        public CgpsTrace(string _MetaUserId, int _MetaTraceId, DateTime _MetaInsertDate, double _lat, double _long, double _Speed, double _Heading,
                            double _RelativeTime, double _SeaLevelAlt, int _NumOfSat, string _GpsTime)
        {
            MetaUserId = _MetaUserId;
            MetaTraceId = _MetaTraceId;
            MetaInsertDate = _MetaInsertDate;
            Lat = _lat;
            Long = _long;
            east = LatLongValue.LatLongToUTM(_lat, _long).utmEx;
            north = LatLongValue.LatLongToUTM(_lat, _long).utmNy;
            utm_zone = LatLongValue.LatLongToUTM(_lat, _long).zone;
            Speed = _Speed;
            Heading = _Heading;
            RelativeTime = _RelativeTime;
            SeaLevelAlt = _SeaLevelAlt;
            NumOfSat = _NumOfSat;
            GpsTime = _GpsTime;
        }

        public CgpsTrace(CgpsTrace gt)
        {
            MetaUserId = gt.MetaUserId;
            MetaTraceId = gt.MetaTraceId;
            MetaInsertDate = gt.MetaInsertDate;
            Lat = gt.Lat;
            Long = gt.Long;
            east = gt.east;
            north = gt.north;
            utm_zone = gt.utm_zone;
            Speed = gt.Speed;
            Heading = gt.Heading;
            RelativeTime = gt.RelativeTime;
            SeaLevelAlt = gt.SeaLevelAlt;
            NumOfSat = gt.NumOfSat;
            GpsTime = gt.GpsTime;
        }
    }

    public class Gaussian
    {
        private Random r;
        private bool use_last_result = false; // flag for NextGaussian3()
        private double y2 = 0.0;  // secondary result for NextGaussian3()

        public Gaussian()
        {
            r = new Random();
        }

        public Gaussian(int seed)
        {
            r = new Random(seed);
        }

        public double NextGaussian3(double mean, double sd) // most efficient
        {
            double x1, x2, w, y1 = 0.0;

            if (use_last_result) // use answer from previous call?
            {
                y1 = y2;
                use_last_result = false;
            }
            else
            {
                do
                {
                    x1 = 2.0 * r.NextDouble() - 1.0;
                    x2 = 2.0 * r.NextDouble() - 1.0;
                    w = (x1 * x1) + (x2 * x2);
                }
                while (w >= 1.0); // are x1 and x2 inside unit circle?

                w = Math.Sqrt((-2.0 * Math.Log(w)) / w);
                y1 = x1 * w;
                y2 = x2 * w;
                use_last_result = true;
            }

            return mean + y1 * sd;
        }
    }


    public partial class CNetwork
    {
        //public static int m_NodeSize = 500, m_LinkSize = 500;
        public int m_TimeInterval = 5, m_TimeIntervalSize = 288;//take 5min as a time interval, from 0 to 1440, total 288 intervals
        public double m_WalkingSpeed = 2.0; // 2mph
        public int m_TimeIntervalTransit = 1, m_TimeIntervalTransitSize = 1440; // public int TimeIntervalAllDay = 1, TimeIntervalAllDaySize = 1440;
        public static int m_MAX_KSP_SIZE = 9;
        public int m_NodeSize, m_LinkSize, m_TransitSize, m_SensorSize;
        public CNode[] aryCNode;
        public CLink[] aryCLink;
        public CShape[] aryCShape;
        public int[,] LinkList;
        public double[] LinkCost;
        public double[] LinkCost_min;
        public double[] LinkCost_max;
        public double[] LinkCost_Penalty;
        public double[] LinkCost_Fuel; // fuel cost in dollar
        public double Fuel_Rate = 0.15; // 0.15 dollar per mile
        public double Transit_Cost = 2.0; // 2 dollar for transit
        public double[][] LinkCost_DayD;
        public double[,] LinkCost_TimeD;
        public double[,] LinkCost_TimeD_Penalty;
        public double[,] LinkCost_TimeD_Transit;
        public Dictionary<int, int> m_NodeIDtoIndex;
        public Dictionary<DirectionalID, int> m_LinkIDtoIndex;
        public Dictionary<int, long> ShapeIDtoIndex;
        public Dictionary<string, int[]> TMCIDtoLink;
        public Dictionary<string, int> TMCwithRealData;
        public string[] TMCIndextoID;
        public Dictionary<int, int> m_SensorIDtoIndex;
        public int[] m_NodeIndextoID;
        public double up, low, left, right;//boundary
        public string city = "";
        public int Global_origin = 0, Global_destination = 0;

        public double[,] TMCHistSpd; // historical speed data for every TMC ID

        //constructor
        public CNetwork()
        {
            m_NodeSize = 0;// m_NodeSize;
            m_LinkSize = 0;// m_LinkSize;
            //CNode[] aryCNode; //aryCNode = new CNode[m_NodeSize];
            //CLink[] aryCLink; //aryCLink = new CLink[m_LinkSize];
        }

        public int[,] InboundLinkAry;
        public int[] InboundLinkSize;
        public int[,] OutgoingLinkAry;
        public int[] OutgoingLinkSize;

        public int[] SEList;
        public int SEList_front, SEList_end;

        public bool ShortestPath_Destination(int destination_index, out int[] PredAry)
        {
            SEList = new int[m_LinkSize];
            int n;
            // test destination index
            if (destination_index >= m_NodeSize)
            {
                PredAry = null;
                return false;
            }
            //
            double[] LabelAry = new double[m_NodeSize];
            double INFINITE = 999999;
            int[] NodeUpdateStatus = new int[m_NodeSize];//0 not updated; 1 updated
            int[] LinkUpdateStatus = new int[m_LinkSize];//0 not in SEList; 1 in SEList
            PredAry = new int[m_NodeSize];
            for (n = 0; n < m_NodeSize; n++)
            {
                LabelAry[n] = INFINITE;
                //NodeUpdateStatus[n] = 0;
                PredAry[n] = -1;
            }

            int node_index, link_index, curr_SEList_front;
            bool curr_Update = false;
            SEList_clear();
            //first push back
            for (n = 0; n < InboundLinkSize[destination_index]; n++)
            {
                link_index = InboundLinkAry[destination_index, n];//inbound link
                NodeUpdateStatus[LinkList[link_index, 1]] = 0;//
                SEList_push_back(link_index);
                LinkUpdateStatus[link_index] = 1;
            }
            LabelAry[destination_index] = 0;
            bool negative_flag = false, loop_flag = false;
            int count = 0;

            while (SEList_front != -1)
            {
                curr_SEList_front = SEList_front;
                node_index = LinkList[SEList_front, 1];
                curr_Update = false;
                loop_flag = false;
                //update label
                if (LinkCost_Penalty[SEList_front] < 0) // detect if negative link cost exists
                {
                    negative_flag = true;
                    //PredAry = null;
                    //return false;
                }
                if (negative_flag)   // prevent loop
                {
                    // detect if current node is on the path from potential Pred node to destination 
                    int nd = LinkList[SEList_front, 0];
                    while (PredAry[nd] != -1)
                    {
                        nd = PredAry[nd];
                        if (nd == node_index)
                        {
                            loop_flag = true;
                            break;
                        }
                    }
                }
                if (LinkCost_Penalty[SEList_front] + LabelAry[LinkList[SEList_front, 0]] < LabelAry[node_index] && !loop_flag)
                {
                    LabelAry[node_index] = LinkCost_Penalty[SEList_front] + LabelAry[LinkList[SEList_front, 0]];
                    PredAry[node_index] = LinkList[SEList_front, 0];
                    curr_Update = true;
                }
                //pop front
                SEList_pop_front();
                LinkUpdateStatus[curr_SEList_front] = 0;
                NodeUpdateStatus[node_index] = 1;
                //push in inbound links of SEList_front to SEList
                if (curr_Update)
                {
                    for (n = 0; n < InboundLinkSize[node_index]; n++)
                    {
                        link_index = InboundLinkAry[node_index, n];//inbound link
                        //if (LinkList[link_index, 1] != LinkList[curr_SEList_front, 0])//do not include the opposite link in direction
                        //{                            
                        if (NodeUpdateStatus[LinkList[link_index, 1]] == 0)
                        {
                            if (LinkUpdateStatus[link_index] == 0)
                            {
                                SEList_push_back(link_index);
                                LinkUpdateStatus[link_index] = 1;
                            }
                        }
                        if (NodeUpdateStatus[LinkList[link_index, 1]] == 1)
                        {
                            if (LinkUpdateStatus[link_index] == 0)
                            {
                                SEList_push_front(link_index);
                                LinkUpdateStatus[link_index] = 1;
                            }
                        }
                        //}
                    }
                }
                //count++;
                //if (count > m_LinkSize * 100)
                //    return false;
            }
            return true;

        }

        /// <summary>
        /// origin based shortest path calculation
        /// </summary>
        /// <param name="origin">origin node ID</param>
        /// <param name="Ref_initial">manually set up an initial value for node label</param>
        /// <param name="PredAry">output: predecessor array</param>
        /// <returns>boolean value indicates if the calculation returns a valid value</returns>
        public bool ShortestPath_Origin(int origin_index, double Ref_initial, out int[] PredAry)
        {
            SEList = new int[m_LinkSize];
            int n;
            //test origin index
            if (origin_index >= m_NodeSize)
            {
                PredAry = null;
                return false;
            }
            //
            double[] LabelAry = new double[m_NodeSize];
            double INFINITE = 999999;
            int[] NodeUpdateStatus = new int[m_NodeSize];//0 not updated; 1 updated
            int[] LinkUpdateStatus = new int[m_LinkSize];//0 not in SEList; 1 in SEList
            PredAry = new int[m_NodeSize];
            for (n = 0; n < m_NodeSize; n++)
            {
                LabelAry[n] = Ref_initial;
                //NodeUpdateStatus[n] = 0;
                PredAry[n] = -1;
            }

            int node_index, link_index, curr_SEList_front;
            bool curr_Update = false;
            SEList_clear();
            //first push back
            for (n = 0; n < OutgoingLinkSize[origin_index]; n++)
            {
                link_index = OutgoingLinkAry[origin_index, n];//outgoing link
                NodeUpdateStatus[LinkList[link_index, 0]] = 0;//
                SEList_push_back(link_index);
                LinkUpdateStatus[link_index] = 1;
            }
            LabelAry[origin_index] = 0;


            while (SEList_front != -1)
            {
                curr_SEList_front = SEList_front;
                node_index = LinkList[SEList_front, 0];
                curr_Update = false;
                //update label
                if (LinkCost_Penalty[SEList_front] < 0)
                {
                    PredAry = null;
                    return false;
                }
                if (LinkCost_Penalty[SEList_front] + LabelAry[LinkList[SEList_front, 1]] < LabelAry[node_index])
                {
                    LabelAry[node_index] = LinkCost_Penalty[SEList_front] + LabelAry[LinkList[SEList_front, 1]];
                    PredAry[node_index] = LinkList[SEList_front, 1];
                    curr_Update = true;
                }
                //pop front
                SEList_pop_front();
                LinkUpdateStatus[curr_SEList_front] = 0;
                NodeUpdateStatus[node_index] = 1;
                //push in outgoing links of SEList_front to SEList
                if (curr_Update)
                {
                    for (n = 0; n < OutgoingLinkSize[node_index]; n++)
                    {
                        link_index = OutgoingLinkAry[node_index, n];//outgoing link
                        //if (LinkList[link_index, 1] != LinkList[curr_SEList_front, 0])//do not include the opposite link in direction
                        //{                            
                        if (NodeUpdateStatus[LinkList[link_index, 0]] == 0)
                        {
                            if (LinkUpdateStatus[link_index] == 0)
                            {
                                SEList_push_back(link_index);
                                LinkUpdateStatus[link_index] = 1;
                            }
                        }
                        if (NodeUpdateStatus[LinkList[link_index, 0]] == 1)
                        {
                            if (LinkUpdateStatus[link_index] == 0)
                            {
                                SEList_push_front(link_index);
                                LinkUpdateStatus[link_index] = 1;
                            }
                        }

                        //}
                    }
                }
            }
            return true;
        }

        private void SEList_clear()
        {
            SEList_front = -1;
            SEList_end = -1;
        }

        private void SEList_push_back(int link)
        {
            if (SEList_front == -1)
            {
                SEList[link] = -1;
                SEList_front = link;
                SEList_end = link;
            }
            else
            {
                SEList[link] = -1;
                SEList[SEList_end] = link;
                SEList_end = link;
            }
        }

        private void SEList_push_front(int link)
        {
            if (SEList_front == -1)
            {
                SEList[link] = -1;
                SEList_front = link;
                SEList_end = link;
            }
            else
            {
                SEList[link] = SEList_front;
                SEList_front = link;
            }
        }

        private void SEList_pop_front()
        {
            int temp_front = SEList_front;
            SEList_front = SEList[temp_front];
            SEList[temp_front] = -1;
            if (SEList_front == -1)
                SEList_end = -1;
        }

        /// <summary>
        /// Sort a list of numbers in order, 
        /// return a integer list containing the original index of numbers, in a sorted order
        /// </summary>
        /// <param name="_ary">input: a list of number in double</param>
        /// <param name="direction">sorting direction: true - decreasing, false - increasing</param>
        /// <returns>the original index of numbers, in a sorted order</returns>
        public int[] SortingFunction(double[] _ary, bool direction)
        {
            //sorting: true - decreasing, false - increasing
            double[] ary = new double[_ary.Length];
            _ary.CopyTo(ary, 0);
            double temp_double;
            int temp_int;
            int[] sort_index = new int[ary.Length];
            for (int k = 0; k < ary.Length; k++)
            {
                sort_index[k] = k;
            }
            //true - decreasing
            if (direction == true)
            {
                for (int i = 0; i < ary.Length; i++)
                {
                    for (int j = i + 1; j < ary.Length; j++)
                    {
                        if (ary[i] < ary[j])
                        {
                            temp_double = ary[i];
                            temp_int = sort_index[i];
                            ary[i] = ary[j];
                            sort_index[i] = sort_index[j];
                            ary[j] = temp_double;
                            sort_index[j] = temp_int;
                        }
                    }
                }
            }
            //false - increasing
            if (direction == false)
            {
                for (int i = 0; i < ary.Length; i++)
                {
                    for (int j = i + 1; j < ary.Length; j++)
                    {
                        if (ary[i] > ary[j])
                        {
                            temp_double = ary[i];
                            temp_int = sort_index[i];
                            ary[i] = ary[j];
                            sort_index[i] = sort_index[j];
                            ary[j] = temp_double;
                            sort_index[j] = temp_int;
                        }
                    }
                }
            }
            return sort_index;
        }

        /// <summary>
        /// Function to get a route object from the result of shortest path calculation
        /// </summary>
        /// <param name="SP_CalculateDirection">determing which SP direction is used, true: destination based; false: origin based</param>
        /// <param name="origin_index">index number of origine node</param>
        /// <param name="destination_index">index number of destination node</param>
        /// <param name="PredAry">the result of SP calculation: a shortest path tree for a certain node</param>
        /// <param name="VoT">value of time</param>
        /// <param name="FlagRealTime">true if the calculation is based on real time travel time</param>
        /// <param name="DepartureTimeInterval">departure time interval</param>
        /// <param name="route">output a CRoute object</param>
        /// <returns>true if route is not empty</returns>
        public bool GetRoute(bool SP_CalculateDirection, int origin_index, int destination_index, int[] PredAry,
                                double VoT, bool FlagRealTime, int DepartureTimeInterval, out CRoute route)//true: destination based; false: origin based
        {
            route = new CRoute();
            //get PathAry
            int node_index = 0;
            int[] PathAryTemp = new int[m_NodeSize];
            int PathNodeSize = 0;

            if (SP_CalculateDirection)//destination based shortest path algorithm
            {
                if (PredAry[origin_index] == -1)
                {
                    route = null;
                    return false;
                }
                node_index = origin_index;
                while (node_index != destination_index)
                {
                    PathAryTemp[PathNodeSize] = node_index;
                    node_index = PredAry[node_index];
                    PathNodeSize++;
                    if (PathNodeSize >= m_NodeSize)
                    {
                        route = null;
                        return false;
                    }
                }
                PathAryTemp[PathNodeSize] = destination_index;
                PathNodeSize++;
                if (PathNodeSize < 2)
                {
                    route = null;
                    return false;
                }
                route.PathAry = new int[PathNodeSize];
                for (int i = 0; i < PathNodeSize; i++)
                {
                    route.PathAry[i] = PathAryTemp[i];
                }
            }
            else //origin based shortest path algorithm
            {
                if (PredAry[destination_index] == -1)
                {
                    route = null;
                    return false;
                }
                node_index = destination_index;
                while (node_index != origin_index)
                {
                    PathAryTemp[PathNodeSize] = node_index;
                    node_index = PredAry[node_index];
                    PathNodeSize++;
                    if (PathNodeSize >= m_NodeSize)
                    {
                        route = null;
                        return false;
                    }
                }
                PathAryTemp[PathNodeSize] = origin_index;
                PathNodeSize++;
                if (PathNodeSize < 2)
                {
                    route = null;
                    return false;
                }
                route.PathAry = new int[PathNodeSize];
                for (int i = 0; i < PathNodeSize; i++)
                {
                    route.PathAry[PathNodeSize - 1 - i] = PathAryTemp[i];
                }
            }


            //get LinkSeq
            int TempNodeID = 0;
            int link_size = 0;
            double temp_link_cost = 0;
            route.LinkSeq = new int[route.PathAry.Length - 1];
            for (int i = 0; i < route.LinkSeq.Length; i++)
            {
                temp_link_cost = 99999999;
                for (int l = 0; l < InboundLinkSize[route.PathAry[i + 1]]; l++)
                {
                    TempNodeID = aryCLink[InboundLinkAry[route.PathAry[i + 1], l]].FromID;
                    if (m_NodeIDtoIndex[TempNodeID] == route.PathAry[i])
                    {
                        link_size++;
                        if (LinkCost_Penalty[InboundLinkAry[route.PathAry[i + 1], l]] < temp_link_cost)
                        {
                            route.LinkSeq[i] = InboundLinkAry[route.PathAry[i + 1], l];
                            temp_link_cost = LinkCost_Penalty[InboundLinkAry[route.PathAry[i + 1], l]];
                        }

                    }
                }
            }
            //if (link_size != route.LinkSeq.Length)
            //{
            //    route = null;
            //    return false;
            //}

            //get travel time
            if (FlagRealTime)
            {
                if (DepartureTimeInterval < 0 || DepartureTimeInterval >= m_TimeIntervalSize)
                {
                    route = null;
                    return false;
                }
                double Departure_Time = DepartureTimeInterval * m_TimeInterval;
                double curr_Time = Departure_Time;
                int curr_Interval = 0;
                for (int j = 0; j < route.LinkSeq.Length; j++)
                {
                    curr_Interval = (int)Math.Floor(curr_Time / m_TimeInterval) % m_TimeIntervalSize;
                    route.PathTime += LinkCost_TimeD[route.LinkSeq[j], curr_Interval];
                    curr_Time += LinkCost_TimeD[route.LinkSeq[j], curr_Interval];
                }
                //TODO: use TrafficColorCode here
                TrafficColorCode(DepartureTimeInterval, route.LinkSeq, out route.TrafficColor);
            }
            else
            {
                for (int j = 0; j < route.LinkSeq.Length; j++)
                {
                    route.PathTime += LinkCost[route.LinkSeq[j]];
                }
                route.TrafficColor = new int[route.LinkSeq.Length];
            }

            //get distance 
            for (int l = 0; l < route.LinkSeq.Length; l++)
            {
                route.PathDist += aryCLink[route.LinkSeq[l]].Link_Length;
            }

            // get cost in dollar
            bool once = true;
            for (int l = 0; l < route.LinkSeq.Length; l++)
            {
                route.PathCost += LinkCost_Fuel[route.LinkSeq[l]];
                if (once == true && aryCLink[route.LinkSeq[l]].Link_Type == 9)
                {
                    route.PathCost += Transit_Cost;
                    once = false;
                }
            }
            route.PathCost += route.PathTime * VoT;

            //get safety factor
            for (int j = 0; j < route.LinkSeq.Length; j++)
            {
                route.PathSafety = route.PathSafety * (1 - aryCLink[route.LinkSeq[j]].Safety);
            }
            route.PathSafety = 1 - route.PathSafety;

            return true;
        }

        private bool UpdateLinkCost_Penalty(int[] LinkSeq, double VOO, double Path_FFTT)
        {
            if (LinkSeq == null)
                return false;
            for (int j = 0; j < LinkSeq.Length; j++)
            {
                LinkCost_Penalty[LinkSeq[j]] = LinkCost[LinkSeq[j]] + VOO * LinkCost[LinkSeq[j]] / Path_FFTT;
            }
            return true;
        }

        private bool UpdateLinkCost_TimeD_Penalty(int[] LinkSeq, double VOO, double Path_FFTT)
        {
            if (LinkSeq == null)
                return false;
            for (int j = 0; j < LinkSeq.Length; j++)
            {
                for (int i = 0; i < m_TimeIntervalSize; i++)
                    LinkCost_TimeD_Penalty[LinkSeq[j], i] = LinkCost_TimeD[LinkSeq[j], i] + VOO * LinkCost_TimeD[LinkSeq[j], i] / Path_FFTT;
            }
            return true;
        }

        public long[][] MappingTable;

        Dictionary<long, string> LinkID2TMC;



        public bool ConvertLatLong2Pix(double max_lat, double max_long, double min_lat, double min_long, double point_lat, double point_long, double pix_width, double pix_heigth, out double point_x, out double point_y)
        {
            if (max_long == min_long || max_lat == min_lat || pix_width == 0 || pix_heigth == 0)
            {
                point_x = -1;
                point_y = -1;
                return false;
            }

            if (point_long < min_long || point_long > max_long || point_lat < min_lat || point_lat > max_lat)
            {
                point_x = -1;
                point_y = -1;
                return false;
            }

            point_x = (point_long - min_long) / (max_long - min_long) * pix_width;
            point_y = (max_lat - point_lat) / (max_lat - min_lat) * pix_heigth;

            return true;

        }

        public int FindClosestNodeIndex(double Lat, double Long) //return node index
        {
            int node_index = -1;
            double dist = 0, max_dist = 1000;
            for (int l = 0; l < m_NodeSize; l++)
            {
                dist = Math.Pow(aryCNode[l].Node_Lat - Lat, 2) + Math.Pow(aryCNode[l].Node_Long - Long, 2);
                if (dist < max_dist && dist < 100)
                {
                    node_index = l;
                    max_dist = dist;
                }
            }
            return node_index;

        }

        public bool ReliablePath(int origin_index, int destination_index)
        {
            int origin = m_NodeIndextoID[origin_index], destination = m_NodeIndextoID[destination_index];
            TextWriter log = new StreamWriter(city + "_log.txt");
            //log.WriteLine("primal \t dual \t mu \t var \t y \t path \t stepsize");
            log.WriteLine("TT\tprimal\tp_mu\tdual\td_mu\tgap\teva_p\tp_mu\teva_d\td_mu\teva_gap\tprimal_quality\tO\tD");

            // Load historical data
            double[] day_MeanVar;
            LoadHistData_CSV("9am", out day_MeanVar);

            #region
            // small sample network
            /*
            double[] variance = new double[3] { 0, 49, 4 }; 
            Random rdm = new Random(123);
            for (int l = 0; l < aryCLink.Length; l++ )
            {
                // aryCLink[l].Link_Variance = variance[l]; // small sample network
                // generate reliability data: SD per mile = 0.8402ln(x) + 0.1387, x is the travel min per mile 
                //aryCLink[l].Link_Variance = Math.Pow((0.8402 * Math.Log(aryCLink[l].Link_FFTT / aryCLink[l].Link_Length) + 0.1387) * aryCLink[l].Link_Length, 2);
                aryCLink[l].Link_Variance = Math.Pow(rdm.NextDouble() * aryCLink[l].Link_FFTT * 10, 2);
            }

            double[][] sample = new double[4][];
            sample[0] = new double[4] { 2, 1, 2, 2 };
            sample[1] = new double[4] { 1, 2, 1, 2 };
            sample[2] = new double[4] { 2, 2, 1, 1 };
            sample[3] = new double[4] { 2, 2, 1, 1 };
             * */
            #endregion
            DateTime start_time, end_time;
            int count = 0;
            int[] PredAry;
            CRoute route = new CRoute();
            double beta = 1.27, lambda = 0.5, UB = 0;
            double mu = 0.01;
            double stepsize = 0;
            double e = 0.00001;
            double y_max = 0, y = 0;
            string str_route = "";
            double primal = 0, dual = 0;
            int iteration = 0, max_iteration = 20;
            double opt_dual = 0, opt_primal = 10000, opt_mu_primal = 0, opt_mu_dual = 0;
            int[] ary_origin = new int[246], ary_dest = new int[246];
            //read OD pairs/////////////////////////////////////////
            FileStream nwfsInput = new FileStream(@"netdata/OD.csv", FileMode.Open, FileAccess.Read);
            StreamReader nwsrInput = new StreamReader(nwfsInput);
            string szSrcLine = "", subStr = "";
            int index = 0, line = 0;
            while ((szSrcLine = nwsrInput.ReadLine()) != null)
            {
                count = 0;
                szSrcLine = szSrcLine.Trim();
                while (szSrcLine.IndexOf(",") != -1)
                {
                    index = szSrcLine.IndexOf(",");
                    subStr = szSrcLine.Substring(0, index);
                    if (count == 0)
                        ary_origin[line] = int.Parse(subStr);
                    else if (count == 1)
                        ary_dest[line] = int.Parse(subStr);
                    count++;
                    szSrcLine = szSrcLine.Substring(index + 1);
                }
                line++;
            }
            nwsrInput.Close();
            nwfsInput.Close();
            ////////////////////////////////////////////////////////

            start_time = DateTime.Now;
            for (int k = 0; k < 246; k++)
            {
                iteration = 0; y_max = 0; UB = 0; mu = 0.001; lambda = 0.5;
                opt_dual = 0; opt_primal = 10000;
                //origin_index = (origin_index + 100) % m_NodeSize;
                //destination_index = (destination_index + 100) % m_NodeSize;
                //origin = m_NodeIndextoID[origin_index]; destination = m_NodeIndextoID[destination_index];
                origin = ary_origin[k]; destination = ary_dest[k];
                origin_index = m_NodeIDtoIndex[origin]; destination_index = m_NodeIDtoIndex[destination];
                // find y_max and UB
                LinkCost.CopyTo(LinkCost_Penalty, 0); // restore link cost array
                if (ShortestPath_Destination(destination_index, out PredAry))
                    GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);

                //if (route != null && route.PathTime >= 45)
                //////////////////////////////////////////////
                {
                    // find UB and ymax
                    for (int l = 0; l < route.LinkSeq.Length; l++)
                    {
                        UB += LinkCost[route.LinkSeq[l]];
                        y_max += aryCLink[route.LinkSeq[l]].TTVariance;
                    }
                    UB += beta * Math.Sqrt(y_max);
                    // y_max = y_max * 0.9;
                    log.Write(Math.Round(route.PathTime, 2) + "\t");

                    ////////////////////////////////////////////////////////
                    // find primal and dual
                    do
                    {
                        primal = 0; dual = 0; str_route = "";
                        // assign link cost
                        for (int l = 0; l < m_LinkSize; l++)
                        {
                            LinkCost_Penalty[l] = LinkCost[l] + mu * aryCLink[l].TTVariance;
                        }
                        // do shortest path search, find path
                        if (ShortestPath_Destination(destination_index, out PredAry))
                            GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
                        else
                            break;
                        // find y
                        if (beta * Math.Sqrt(y_max) - mu * y_max > 0)
                            y = 0;
                        else
                            y = y_max;
                        // calculate primal and dual function value
                        double total_var = 0;
                        for (int l = 0; l < route.LinkSeq.Length; l++)
                        {
                            primal += LinkCost[route.LinkSeq[l]];
                            total_var += aryCLink[route.LinkSeq[l]].TTVariance;
                            dual += LinkCost_Penalty[route.LinkSeq[l]];
                            str_route += aryCLink[route.LinkSeq[l]].ID.ID.ToString() + ",";
                        }
                        primal += beta * Math.Sqrt(total_var);
                        dual += (beta * Math.Sqrt(y) - mu * y);

                        if (dual > UB)
                            UB = dual;

                        // find optimal
                        if (dual >= opt_dual)
                        {
                            opt_dual = dual;
                            opt_mu_dual = mu;
                        }
                        if (primal <= opt_primal)
                        {
                            opt_primal = primal;
                            opt_mu_primal = mu;
                        }

                        // output
                        //log.WriteLine(Math.Round(primal, 3) + "\t" + Math.Round(dual, 3) + "\t" + Math.Round(mu, 3) + "\t" + Math.Round(total_var, 3) + "\t" + Math.Round(y, 3)
                        //    + "\t" + route.PathTime + "\t" + stepsize);
                        //Console.WriteLine(Math.Round(primal, 3) + "\t" + Math.Round(dual, 3) + "\t" + Math.Round(mu, 3)
                        //    + "\t" + str_route + "\t" + Math.Round(stepsize, 3));

                        // calculate mu(k+1)
                        double temp = 0;
                        temp = Math.Pow(total_var - y, 2);

                        if (temp == 0)
                            temp = 0.0000001;
                        lambda = 1 / (1 / lambda + 1); // lambda = 1 / ((double)iteration + 1);
                        stepsize = lambda * (UB - dual) * (total_var - y) / temp;
                        if (stepsize == 0)
                        {
                            lambda = 0.5;
                            stepsize = lambda;
                        }
                        mu += stepsize;
                        // mu += 0.1;
                        if (mu < 0)
                            mu = 0;

                        iteration++;
                    }
                    while (iteration < max_iteration);
                    log.Write(Math.Round(opt_primal, 3) + "\t" + Math.Round(opt_mu_primal, 3) + "\t" + Math.Round(opt_dual, 3) + "\t" + Math.Round(opt_mu_dual, 3)
                        + "\t" + Math.Round(1 - opt_dual / opt_primal, 4) + "\n");

                    //log.WriteLine("\n");
                    //log.WriteLine("UB: " + UB + "\t y max: " + y_max + "\t mu bound: " + 1 / Math.Sqrt(y_max) + "\t Opt Dual: " + Math.Round(opt_dual, 3)
                    //    + "\t Opt Dual Mu: " + Math.Round(opt_mu_dual, 3) + "\t Opt Primal: " + Math.Round(opt_primal, 3) + "\t Opt Primal Mu: " + Math.Round(opt_mu_primal, 3) + "\n");
                    // generate reliability data: SD per mile = 0.8402ln(x) + 0.1387, x is the travel min per mile 

                    /*///////////////////////////////////////////////////////
                    // evaluation: use grid search to find minimum of primal
                    int eva_max_iteration = 100;
                    double eva_opt_dual = 0, eva_opt_primal = 10000, eva_opt_mu_primal = 0, eva_opt_mu_dual = 0;
                    iteration = 0; mu = 0;
                    do
                    {
                        primal = 0; dual = 0; str_route = "";
                        // assign link cost
                        for (int l = 0; l < m_LinkSize; l++)
                        {
                            LinkCost_Penalty[l] = LinkCost[l] + mu * aryCLink[l].TTVariance;
                        }
                        // do shortest path search, find path
                        if (ShortestPath_Destination(destination, out PredAry))
                            GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
                        else
                            break;
                        // find y
                        if (beta * Math.Sqrt(y_max) - mu * y_max > 0)
                            y = 0;
                        else
                            y = y_max;
                        // calculate primal and dual function value
                        double total_var = 0;
                        for (int l = 0; l < route.LinkSeq.Length; l++)
                        {
                            primal += LinkCost[route.LinkSeq[l]];
                            total_var += aryCLink[route.LinkSeq[l]].TTVariance;
                            dual += LinkCost_Penalty[route.LinkSeq[l]];
                            str_route += aryCLink[route.LinkSeq[l]].ID.ID.ToString() + ",";
                        }
                        primal += beta * Math.Sqrt(total_var);
                        dual += (beta * Math.Sqrt(y) - mu * y);

                        if (dual > UB)
                            UB = dual;

                        // find optimal
                        if (dual >= eva_opt_dual)
                        {
                            eva_opt_dual = dual;
                            eva_opt_mu_dual = mu;
                        }
                        if (primal <= eva_opt_primal)
                        {
                            eva_opt_primal = primal;
                            eva_opt_mu_primal = mu;
                        }

                        // output
                        //log.WriteLine(Math.Round(primal, 3) + "\t" + Math.Round(dual, 3) + "\t" + Math.Round(mu, 3) + "\t" + Math.Round(total_var, 3) + "\t" + Math.Round(y, 3)
                        //    + "\t" + route.PathTime + "\t" + stepsize);
                        //Console.WriteLine(Math.Round(primal, 3) + "\t" + Math.Round(dual, 3) + "\t" + Math.Round(mu, 3)
                        //    + "\t" + str_route + "\t" + Math.Round(stepsize, 3));

                        mu += 0.01;

                        iteration++;
                    }
                    while (iteration < eva_max_iteration);
                    log.Write(Math.Round(eva_opt_primal, 3) + "\t" + Math.Round(eva_opt_mu_primal, 3) + "\t" + Math.Round(eva_opt_dual, 3) + "\t" + Math.Round(eva_opt_mu_dual, 3)
                        + "\t" + Math.Round(1 - eva_opt_dual / eva_opt_primal, 3) + "\t" + Math.Round(eva_opt_primal / opt_primal, 3) + "\t" + origin + "\t" + destination + "\n");
                    ///////////////////////////////////////////////////////////////*/
                }
            }
            end_time = DateTime.Now;
            log.Close();
            return true;
        }

        public bool draw()
        {
            //// test/////////////////////////////////////////
            //int[] Origin = new int[10];// { 49553, 151094, 144842, 136054, 66486, 81425, 140124, 92225, 1629, 198842 };
            //int[] Dest = new int[10];// { 65616, 15843, 12310, 96413, 21513, 167667, 151766, 143867, 77792, 149992 };
            //double[] o_Lat = new double[] { 37.27716, 37.81234, 37.70825, 38.14645, 37.71567, 37.51311, 37.82154, 37.90178, 37.35696, 37.37854 };
            //double[] o_Log = new double[] { -122.00718, -122.30308, -122.40526, -122.21658, -122.39911, -122.33108, -122.29766, -122.30563, -121.84681, -122.06846 };
            //double[] d_Lat = new double[] { 37.75044, 37.36404, 37.29196, 37.74277, 37.36209, 37.83235, 37.4447, 37.48416, 37.70501, 37.83723 };
            //double[] d_Log = new double[] { -122.4028, -121.90178, -122.01545, -122.17389, -121.8548, -121.98233, -122.16904, -122.22885, -122.46958, -122.26215 };
            //for (int l = 0; l < 10; l++)
            //{
            //    Origin[l] = FindClosestNode(o_Lat[l], o_Log[l]);
            //    Dest[l] = FindClosestNode(d_Lat[l], d_Log[l]);
            //    //o_Lat[l] = aryCNode[m_NodeIDtoIndex[Origin[l]]].Node_Lat;
            //    //o_Log[l] = aryCNode[m_NodeIDtoIndex[Origin[l]]].Node_Long;
            //    //d_Lat[l] = aryCNode[m_NodeIDtoIndex[Dest[l]]].Node_Lat;
            //    //d_Log[l] = aryCNode[m_NodeIDtoIndex[Dest[l]]].Node_Long;
            //}
            ////////////////////////////////////////////////

            int[] BestSensor = new int[] { 1, 2, 10, 30 };
            double[] day_MeanVar;
            LoadHistData_CSV("9am", out day_MeanVar);
            //Bitmap bmp = DrawSensor(2000, BestSensor, BestSensor);
            Bitmap bmp = demo_1(2000);
            DateTime time = DateTime.Now;
            string filename = time.Minute.ToString() + "_img.bmp";
            bmp.Save(filename);
            bmp.Dispose();

            return true;
        }

        public bool ReliablePath_Sampling(int origin_index, int destination_index)
        {
            int origin = m_NodeIndextoID[origin_index], destination = m_NodeIndextoID[destination_index];
            TextWriter log = new StreamWriter(city + "_Sampling_log.txt");
            //log.WriteLine("primal \t dual \t mu \t var \t y \t path \t stepsize");
            log.WriteLine("TT\tprimal\tdual\tgap\teva_p\teva_d\teva_gap\tprimal_quality\tO\tD");

            // Load historical data            
            double[] day_MeanVar;
            LoadHistData_CSV("9am", out day_MeanVar);
            //LoadHistData_CSV("9am_Demo");

            DateTime start_time, end_time;
            int day_size = TMCHistSpd.GetLength(1);
            int tmc_index = 0, tmc_size = TMCHistSpd.GetLength(0);
            double tt = 0, var = 0, length = 0, spd = 0, MeanTTvar = 0;
            int count = 0;
            int[] PredAry;
            CRoute route = new CRoute();
            double UB = 0, beta = 1.27; // 4; // 1.27;
            double[] mu = new double[day_size], stepsize_mu = new double[day_size], lambda_mu = new double[day_size];
            double nu = 0.01, stepsize_nu = 0, lambda_nu = 0.5;
            double y_max = 0, y = 0;
            double[] w = new double[day_size];
            double[] day_cost = new double[day_size];
            double average_cost = 0;

            string str_route = "", opt_str_route = "";
            double primal = 0, dual = 0, gap = 0;
            int iteration = 0, max_iteration = 10;
            double opt_dual = 0, opt_primal = 10000, opt_gap = 10000;
            double initial = 0.0001;
            double coverage = 0;
            Random rdm = new Random(123);
            double[] ary_dual = new double[max_iteration];
            //read OD pairs/////////////////////////////////////////
            int[] ary_origin = new int[246], ary_dest = new int[246];
            FileStream nwfsInput = new FileStream(@"netdata/OD.csv", FileMode.Open, FileAccess.Read);
            StreamReader nwsrInput = new StreamReader(nwfsInput);
            string szSrcLine = "", subStr = "";
            int index = 0, line = 0;
            while ((szSrcLine = nwsrInput.ReadLine()) != null)
            {
                count = 0;
                szSrcLine = szSrcLine.Trim();
                while (szSrcLine.IndexOf(",") != -1)
                {
                    index = szSrcLine.IndexOf(",");
                    subStr = szSrcLine.Substring(0, index);
                    if (count == 0)
                        ary_origin[line] = int.Parse(subStr);
                    else if (count == 1)
                        ary_dest[line] = int.Parse(subStr);
                    count++;
                    szSrcLine = szSrcLine.Substring(index + 1);
                }
                line++;
            }
            nwsrInput.Close();
            nwfsInput.Close();
            ////////////////////////////////////////////////////////
            /*for (int k = 0; k < 1000; k++)
            {
                iteration = 0; lambda_nu = 0.01; nu = initial;
                coverage = 0;
                for (int d = 0; d < day_size; d++)
                {
                    mu[d] = initial / day_size;
                    lambda_mu[d] = 0.01;
                }
                opt_dual = 0; opt_primal = 10000; opt_gap = 10000; opt_str_route = "";
                origin_index = (origin_index + 100) % m_NodeSize;
                destination_index = (destination_index + 100) % m_NodeSize;
                //origin_index = (int)(rdm.NextDouble() * (m_NodeSize - 1));
                //destination_index = (int)(rdm.NextDouble() * (m_NodeSize - 1));
                //origin = m_NodeIndextoID[origin_index]; destination = m_NodeIndextoID[destination_index];
                // find y_max and UB
                LinkCost.CopyTo(LinkCost_Penalty, 0);
                if (ShortestPath_Destination(destination, out PredAry))
                    GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);

                if (route != null && route.PathTime >= 45)
                {
                    double c = 0;
                    for (int l = 0; l < route.LinkSeq.Length; l++)
                        if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                            c += aryCLink[route.LinkSeq[l]].Link_Length;
                    coverage = c / route.PathDist;
                }

                if (coverage > 0.3)
            */
            start_time = DateTime.Now;
            for (int k = 24; k < 50; k++) //246
            {
                //log.Close();
                //log = new StreamWriter(city + "_Sampling_log.txt"); // new log

                iteration = 0; lambda_nu = 0.01; nu = 0.1;
                coverage = 0;
                for (int d = 0; d < day_size; d++)
                {
                    mu[d] = 1.0 / day_size;
                    lambda_mu[d] = 0.01;
                }
                opt_dual = 0; opt_primal = 10000; opt_gap = 10000; opt_str_route = "";

                origin = ary_origin[k]; destination = ary_dest[k];
                origin_index = m_NodeIDtoIndex[origin]; destination_index = m_NodeIDtoIndex[destination];
                // find y_max and UB
                LinkCost.CopyTo(LinkCost_Penalty, 0);
                if (ShortestPath_Destination(destination_index, out PredAry))
                    GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);

                ////////////////////////////////////////////////
                {
                    // find UB and ymax
                    day_cost = new double[day_size]; y_max = 0; UB = 0;
                    for (int l = 0; l < route.LinkSeq.Length; l++)
                    {
                        UB += LinkCost[route.LinkSeq[l]];
                        if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                        {
                            tmc_index = TMCwithRealData[aryCLink[route.LinkSeq[l]].TMC];
                            for (int d = 0; d < day_size; d++)
                            {
                                day_cost[d] += aryCLink[route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                            }
                        }
                        else
                        {
                            for (int d = 0; d < day_size; d++)
                            {
                                day_cost[d] += aryCLink[route.LinkSeq[l]].TravelTime;
                            }
                        }
                    }

                    for (int d = 0; d < day_size; d++)
                    {
                        y_max += Math.Pow(day_cost[d] - UB, 2);
                    }
                    y_max = y_max / (day_size - 1);
                    UB += beta * Math.Sqrt(y_max);
                    // y_max = y_max * 0.9;
                    log.Write(Math.Round(route.PathTime, 2) + "\t");

                    ////////////////////////////////////////////////////////
                    // find primal and dual
                    do
                    {
                        primal = 0; dual = 0; str_route = ""; gap = 0;
                        day_cost = new double[day_size];
                        average_cost = 0;
                        double total_var = 0, total_w = 0;
                        // assign link cost
                        double temp = 0;
                        bool assign = true;
                        Gaussian g;
                        double mean = 0, sd = 0, g_value = 0;
                        while (assign)
                        {
                            assign = false;
                            for (int l = 0; l < m_LinkSize; l++)
                            {
                                if (aryCLink[l].TMC != null && TMCwithRealData.ContainsKey(aryCLink[l].TMC))
                                {
                                    temp = 0;
                                    tmc_index = TMCwithRealData[aryCLink[l].TMC];
                                    for (int d = 0; d < day_size; d++)
                                    {
                                        temp += mu[d] * (aryCLink[l].Link_Length / TMCHistSpd[tmc_index, d] * 60 - LinkCost[l]);
                                    }
                                    LinkCost_Penalty[l] = LinkCost[l] + temp;
                                    if (LinkCost_Penalty[l] <= 0)  // reduce values of mu in scale
                                    {
                                        //double mu_sum = 0;
                                        for (int d = 0; d < day_size; d++)
                                        {
                                            //mu[d] = mu[d] * 0.9;
                                            mu[d] = mu[d] - stepsize_mu[d] + stepsize_mu[d] * 0.9;
                                        }
                                        //Console.WriteLine(mu_sum);
                                        assign = true;
                                        break;
                                    }
                                }
                                else
                                {
                                    g = new Gaussian(l);
                                    mean = LinkCost[l];
                                    sd = 0;
                                    g_value = 0;
                                    temp = 0;
                                    length = aryCLink[l].Link_Length;
                                    spd = aryCLink[l].SpeedLimit;
                                    for (int d = 0; d < day_size; d++)
                                    {
                                        sd = Math.Sqrt(day_MeanVar[d]) * (length / spd * 60);
                                        g_value = g.NextGaussian3(mean, sd);
                                        temp += mu[d] * (g_value - mean);
                                    }
                                    LinkCost_Penalty[l] = LinkCost[l] + temp;
                                    if (LinkCost_Penalty[l] <= 0)  // reduce values of mu in scale
                                    {
                                        for (int d = 0; d < day_size; d++)
                                        {
                                            //mu[d] = mu[d] * 0.9;
                                            mu[d] = mu[d] - stepsize_mu[d] + stepsize_mu[d] * 0.9;
                                        }
                                        assign = true;
                                        break;
                                    }
                                }
                            }
                        }
                        //Console.WriteLine(iteration + "==================");
                        // do shortest path search, find path, solve Lx
                        if (ShortestPath_Destination(destination_index, out PredAry))
                            GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
                        else
                            break;
                        // solve Lw
                        for (int d = 0; d < day_size; d++)
                        {
                            w[d] = mu[d] * (day_size - 1) / (2 * nu);
                        }

                        // find y, solve Ly
                        if (beta * Math.Sqrt(y_max) - nu * y_max > 0)
                            y = 0;
                        else
                            y = y_max;
                        // calculate primal and dual function value
                        for (int l = 0; l < route.LinkSeq.Length; l++)
                        {
                            average_cost += LinkCost[route.LinkSeq[l]];
                            // calculate travel time for each sample
                            if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                            {
                                tmc_index = TMCwithRealData[aryCLink[route.LinkSeq[l]].TMC];
                                for (int d = 0; d < day_size; d++)
                                {
                                    day_cost[d] += aryCLink[route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                                }
                            }
                            else
                            {
                                for (int d = 0; d < day_size; d++)
                                {
                                    day_cost[d] += LinkCost[route.LinkSeq[l]];
                                }
                            }
                            //else
                            //{
                            //    g = new Gaussian(route.LinkSeq[l]);
                            //    mean = LinkCost[route.LinkSeq[l]];
                            //    sd = 0; g_value = 0;
                            //    length = aryCLink[route.LinkSeq[l]].Link_Length;
                            //    spd = aryCLink[route.LinkSeq[l]].SpeedLimit;
                            //    double[] g_ary = new double[day_size];
                            //    for (int d = 0; d < day_size; d++)
                            //    {
                            //        sd = Math.Sqrt(day_MeanVar[d]) * (length / spd * 60);
                            //        g_value = g.NextGaussian3(mean, sd);
                            //        day_cost[d] += g_value;
                            //        g_ary[d] = g_value;
                            //    }
                            //}
                            dual += LinkCost_Penalty[route.LinkSeq[l]];
                            str_route += aryCLink[route.LinkSeq[l]].ID.ID.ToString() + ",";
                        }
                        for (int d = 0; d < day_size; d++)
                        {
                            total_var += Math.Pow(day_cost[d] - average_cost, 2);
                            total_w += (nu / (day_size - 1) * w[d] - mu[d]) * w[d];
                        }
                        primal = average_cost + beta * Math.Sqrt(total_var / (day_size - 1));
                        dual += total_w + beta * Math.Sqrt(y) - nu * y;
                        ary_dual[iteration] = dual;

                        if (dual > UB)
                            UB = dual;
                        if (gap < 0)
                            gap = 0;

                        // find optimal
                        if (dual >= opt_dual)
                        {
                            opt_dual = dual;
                        }
                        if (primal <= opt_primal)
                        {
                            opt_primal = primal;
                            UB = primal;
                            opt_str_route = str_route;
                        }
                        gap = opt_primal - opt_dual;
                        if (gap <= opt_gap)
                        {
                            opt_gap = gap;
                        }

                        // output
                        //log.WriteLine(Math.Round(primal, 3) + "\t" + Math.Round(dual, 3) + "\t" + Math.Round(mu, 3) + "\t" + Math.Round(total_var, 3) + "\t" + Math.Round(y, 3)
                        //    + "\t" + route.PathTime + "\t" + stepsize);
                        //Console.WriteLine(Math.Round(primal, 3) + "\t" + Math.Round(dual, 3) + "\t" + Math.Round(mu, 3)
                        //    + "\t" + str_route + "\t" + Math.Round(stepsize, 3));

                        // calculate mu(k+1) and nu(k+1) with subgradient
                        temp = 0; total_w = 0;
                        for (int d = 0; d < day_size; d++)
                        {
                            temp = (day_cost[d] - average_cost - w[d]);
                            temp = Math.Abs(temp);
                            if (temp == 0)
                                temp = 0.0001;
                            stepsize_mu[d] = lambda_mu[d] * (UB - dual) / temp;
                            lambda_mu[d] = 1 / (1 / lambda_mu[d] + 1);
                            if (stepsize_mu[d] == 0)
                            {
                                lambda_mu[d] = 0.01;
                                stepsize_mu[d] = lambda_mu[d];
                            }
                            total_w += Math.Pow(w[d], 2);
                        }
                        temp = total_w / (day_size - 1) - y;
                        if (temp == 0)
                            temp = 0.0001;
                        stepsize_nu = lambda_nu * (UB - dual) / temp;
                        lambda_nu = 1 / (1 / lambda_nu + 1);
                        if (stepsize_nu == 0)
                        {
                            lambda_nu = 0.01;
                            stepsize_nu = lambda_nu;
                        }
                        //double mu_sum = 0;
                        for (int d = 0; d < day_size; d++)
                        {
                            mu[d] += stepsize_mu[d];
                            if (mu[d] < 0)
                                mu[d] = 0;
                            //mu_sum += mu[d];
                        }
                        //while (mu_sum >= 4)
                        //{
                        //    mu_sum = 0;
                        //    for (int d = 0; d < day_size; d++)
                        //    {
                        //        mu[d] = mu[d] * 0.9; // / day_size;
                        //        mu_sum += mu[d];
                        //    }
                        //}

                        nu += stepsize_nu;
                        if (nu < 0)
                            nu = 0.0001;

                        iteration++;
                        //log.Write(Math.Round(mu[0], 3) + "\t" + Math.Round(mu[1], 3) + "\t" + Math.Round(mu[2], 3) + "\t" +
                        //    Math.Round(mu[3], 3) + "\t" + Math.Round(nu, 3) + "\t" + Math.Round(primal, 3) + "\t" + Math.Round(dual, 3)
                        //     + "\t" + Math.Round(opt_primal, 3) + "\t" + Math.Round(opt_dual, 3) + "\n"); 
                    }
                    while (iteration < max_iteration);
                    /////////////////////////////////////////////////////
                    log.Write(Math.Round(opt_primal, 3) + "\t" + Math.Round(opt_dual, 3) + "\t" + Math.Round(1 - opt_dual / opt_primal, 4)
                        + "\t" + origin + "\t" + destination + "\t" + coverage + "\t" + "\n");
                    //log.Close();

                    //log.WriteLine("\n");
                    //log.WriteLine("UB: " + UB + "\t y max: " + y_max + "\t mu bound: " + 1 / Math.Sqrt(y_max) + "\t Opt Dual: " + Math.Round(opt_dual, 3)
                    //    + "\t Opt Dual Mu: " + Math.Round(opt_mu_dual, 3) + "\t Opt Primal: " + Math.Round(opt_primal, 3) + "\t Opt Primal Mu: " + Math.Round(opt_mu_primal, 3) + "\n");
                    // generate reliability data: SD per mile = 0.8402ln(x) + 0.1387, x is the travel min per mile 


                }
                ///////////////////////////////////////////////////

                // evaluation with Random Draw path enumeration method 
                {
                    Gaussian g;
                    double mean = 0;
                    double sd = 0;
                    double g_value = 0;
                    double total_var = 0;

                    /*/ individual day reliable path ////////////////////////////////
                    primal = 0;
                    opt_primal = 1000000;
                    for (int day = 0; day < day_size; day++)
                    {
                        // individual day shortest path
                        for (int l = 0; l < m_LinkSize; l++)
                        {
                            if (aryCLink[l].TMC != null && TMCwithRealData.ContainsKey(aryCLink[l].TMC))
                            {
                                tmc_index = TMCwithRealData[aryCLink[l].TMC];
                                LinkCost_Penalty[l] = aryCLink[l].Link_Length / TMCHistSpd[tmc_index, day] * 60;
                            }
                        }
                        if (ShortestPath_Destination(destination_index, out PredAry))
                            GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
                        average_cost = 0; total_var = 0;
                        day_cost = new double[day_size];
                        int route_length = route.LinkSeq.Length;
                        for (int l = 0; l < route_length; l++)
                        {
                            average_cost += LinkCost[route.LinkSeq[l]];
                            // calculate travel time for each sample
                            if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                            {
                                tmc_index = TMCwithRealData[aryCLink[route.LinkSeq[l]].TMC];
                                for (int d = 0; d < day_size; d++)
                                {
                                    day_cost[d] += aryCLink[route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                                }
                            }
                            else
                            {
                                for (int d = 0; d < day_size; d++)
                                {
                                    day_cost[d] += LinkCost[route.LinkSeq[l]];
                                }
                            }
                        }
                        for (int d = 0; d < day_size; d++)
                        {
                            total_var += Math.Pow(day_cost[d] - average_cost, 2);
                        }
                        primal = average_cost + beta * Math.Sqrt(total_var / (day_size - 1));
                        // find optimal
                        if (primal <= opt_primal)
                        {
                            opt_primal = primal;
                        }
                        // log.Write("Individual day" + Math.Round(primal, 3) + "\t" + Math.Round(opt_primal, 3) + "\n");
                    }
                    log.Write("Individual day " + Math.Round(opt_primal, 3) + "\t");
                    //////////////////////////////////////////////*/

                    primal = 0;
                    opt_primal = 1000000;
                    /*/ Random Draw ////////////////////////////////////                    
                    for (int i = 0; i < 20; i++)
                    {
                        g = new Gaussian(i);
                        average_cost = 0; total_var = 0;
                        day_cost = new double[day_size];
                        if (i >= 0)
                            for (int l = 0; l < m_LinkSize; l++)
                            {                                
                                mean = LinkCost[l];
                                sd = Math.Sqrt(mean);
                                g_value = g.NextGaussian3(mean, sd);
                                LinkCost_Penalty[l] = Math.Abs(g_value);
                            }
                        if (ShortestPath_Destination(destination_index, out PredAry))
                            GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
                        // calculate primal and dual function value
                        for (int l = 0; l < route.LinkSeq.Length; l++)
                        {
                            average_cost += LinkCost[route.LinkSeq[l]];
                            // calculate travel time for each sample
                            if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                            {
                                tmc_index = TMCwithRealData[aryCLink[route.LinkSeq[l]].TMC];
                                for (int d = 0; d < day_size; d++)
                                {
                                    day_cost[d] += aryCLink[route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                                }
                            }
                            else
                            {
                                for (int d = 0; d < day_size; d++)
                                {
                                    day_cost[d] += LinkCost[route.LinkSeq[l]];
                                }
                            }
                        }
                        for (int d = 0; d < day_size; d++)
                        {
                            total_var += Math.Pow(day_cost[d] - average_cost, 2);
                        }
                        primal = average_cost + beta * Math.Sqrt(total_var / (day_size - 1));
                        // find optimal
                        if (primal <= opt_primal)
                        {
                            opt_primal = primal;
                        }
                        // log.Write("Random Draw " + Math.Round(primal, 3) + "\t" + Math.Round(opt_primal, 3) + "\n");
                    }
                    log.Write("Random Draw " + Math.Round(opt_primal, 3) + "\n");
                    //////////////////////////////////////////////////*/
                }
                //log.Close();
                //////////////////////////////////////////////////////
            }
            end_time = DateTime.Now;
            log.Close();
            return true;
        }

        public void ReliablePathEvaluation()
        {
            // read in path, find best UB value
            int[] Origin_ID = new int[] { 52292, 151094, 136822, 136054, 143655, 81422, 140124, 109800, 139476 };
            int[] Dest_ID = new int[] { 154600, 15843, 47948, 148424, 52057, 201951, 240663, 136327, 135960 };
            /////////////////////////////////////////////////////////
            // find best primal value from path enumeration results
            double[] day_MeanVar;
            LoadHistData_CSV("9am", out day_MeanVar);
            int day_size = TMCHistSpd.GetLength(1);
            int tmc_index = 0;
            double beta = 1.27;
            FileStream nwfsInput;
            StreamReader nwsrInput;
            string szSrcLine = "";
            int index = 0, route_length = 0, count = 0;
            CRoute E_route = new CRoute();
            int[] note_seq, link_seq;
            string[] raw_data;
            double[] best_primal = new double[9];
            for (int i = 0; i < 9; i++)
            {
                nwfsInput = new FileStream(@"C:\Research\Reliability\More experiments\path_set_" + i + ".csv", FileMode.Open, FileAccess.Read);
                nwsrInput = new StreamReader(nwfsInput);
                best_primal[i] = 999999;
                while ((szSrcLine = nwsrInput.ReadLine()) != null)
                {
                    index = szSrcLine.IndexOf(",");
                    szSrcLine = szSrcLine.Substring(index + 1);
                    index = szSrcLine.IndexOf(",");
                    route_length = int.Parse(szSrcLine.Substring(0, index));
                    szSrcLine = szSrcLine.Substring(index + 1);
                    raw_data = szSrcLine.Split(',');
                    note_seq = new int[route_length];
                    link_seq = new int[route_length - 1];
                    E_route = new CRoute(note_seq, link_seq, link_seq, 0, 0, 0, 0);
                    count++;
                    for (int n = 0; n < route_length; n++)
                    {
                        E_route.PathAry[n] = m_NodeIDtoIndex[int.Parse(raw_data[n])];
                    }
                    //get LinkSeq
                    int TempNodeID = 0;
                    int link_size = 0;
                    double temp_link_cost = 0;
                    for (int k = 0; k < route_length - 1; k++)
                    {
                        temp_link_cost = 99999999;
                        for (int l = 0; l < InboundLinkSize[E_route.PathAry[k + 1]]; l++)
                        {
                            TempNodeID = aryCLink[InboundLinkAry[E_route.PathAry[k + 1], l]].FromID;
                            if (m_NodeIDtoIndex[TempNodeID] == E_route.PathAry[k])
                            {
                                link_size++;
                                if (LinkCost_Penalty[InboundLinkAry[E_route.PathAry[k + 1], l]] < temp_link_cost)
                                {
                                    E_route.LinkSeq[k] = InboundLinkAry[E_route.PathAry[k + 1], l];
                                    temp_link_cost = LinkCost_Penalty[InboundLinkAry[E_route.PathAry[k + 1], l]];
                                }

                            }
                        }
                    }

                    // calculate primal
                    double primal = 0;
                    double[] day_cost = new double[day_size];
                    double average_cost = 0;
                    double total_var = 0;
                    for (int l = 0; l < E_route.LinkSeq.Length; l++)
                    {
                        average_cost += LinkCost[E_route.LinkSeq[l]];
                        // calculate travel time for each sample
                        if (aryCLink[E_route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[E_route.LinkSeq[l]].TMC))
                        {
                            tmc_index = TMCwithRealData[aryCLink[E_route.LinkSeq[l]].TMC];
                            for (int d = 0; d < day_size; d++)
                            {
                                day_cost[d] += aryCLink[E_route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                            }
                        }
                        else
                        {
                            for (int d = 0; d < day_size; d++)
                            {
                                day_cost[d] += aryCLink[E_route.LinkSeq[l]].TravelTime;
                            }
                        }
                    }
                    for (int d = 0; d < day_size; d++)
                    {
                        total_var += Math.Pow(day_cost[d] - average_cost, 2);
                    }
                    primal = average_cost + beta * Math.Sqrt(total_var / (day_size - 1));
                    if (primal <= best_primal[i])
                        best_primal[i] = primal;
                }
                nwsrInput.Close();
                nwfsInput.Close();
            }
            ////////////////////////////////////////////////////////

            // use ReliablePath_Sampling_Single to find most reliable path
            double[] LR_best_primal = new double[9];
            for (int i = 0; i < 9; i++)
            {
                LR_best_primal[i] = ReliablePath_Sampling_Single(m_NodeIDtoIndex[Origin_ID[i]], m_NodeIDtoIndex[Dest_ID[i]], 10);
            }
            // calculate stats

            return;

        }

        public double ReliablePath_Sampling_Single(int origin_index, int destination_index, int max_iteration)
        {
            TextWriter log = new StreamWriter(city + "_Sampling_Single_log.txt");
            log.WriteLine("iteration\tprimal\tdual\tgap");

            // Load historical data            
            double[] day_MeanVar;
            LoadHistData_CSV("9am", out day_MeanVar);
            //LoadHistData_CSV("9am_newDEMO", out day_MeanVar);

            int day_size = TMCHistSpd.GetLength(1);
            int tmc_index = 0, tmc_size = TMCHistSpd.GetLength(0);
            double length = 0, spd = 0;
            int[] PredAry;
            CRoute route = new CRoute();
            double beta = 1; //beta = 1.27;
            double UB = 0;
            double[] mu = new double[day_size], stepsize_mu = new double[day_size], lambda_mu = new double[day_size];
            double nu = 0.01, stepsize_nu = 0, lambda_nu = 0.5;
            double y_max = 0, y = 0;
            double[] w = new double[day_size];
            double[] day_cost = new double[day_size];
            double average_cost = 0;

            string str_route = "", opt_str_route = "";
            double primal = 0, dual = 0, gap = 0;
            int iteration = 0;
            double opt_dual = 0, opt_primal = 10000, opt_gap = 10000;
            Random rdm = new Random(123);
            double[] ary_dual = new double[max_iteration];

            // find y_max and UB
            LinkCost.CopyTo(LinkCost_Penalty, 0);
            if (ShortestPath_Destination(destination_index, out PredAry))
                GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
            else
                return -1;


            // find UB and ymax
            day_cost = new double[day_size]; y_max = 0; UB = 0;
            for (int l = 0; l < route.LinkSeq.Length; l++)
            {
                UB += LinkCost[route.LinkSeq[l]];
                if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                {
                    tmc_index = TMCwithRealData[aryCLink[route.LinkSeq[l]].TMC];
                    for (int d = 0; d < day_size; d++)
                    {
                        day_cost[d] += aryCLink[route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                    }
                }
                else
                {
                    for (int d = 0; d < day_size; d++)
                    {
                        day_cost[d] += aryCLink[route.LinkSeq[l]].TravelTime;
                    }
                }
            }

            for (int d = 0; d < day_size; d++)
            {
                y_max += Math.Pow(day_cost[d] - UB, 2);
            }
            y_max = y_max / (day_size - 1);
            UB += beta * Math.Sqrt(y_max);
            log.Write(Math.Round(route.PathTime, 2) + "\t");



            ////////////////////////////////////////////////////////
            // find primal and dual
            do
            {
                primal = 0; dual = 0; str_route = ""; gap = 0;
                day_cost = new double[day_size];
                average_cost = 0;
                double total_var = 0, total_w = 0;
                // assign link cost
                double temp = 0;
                bool assign = true;
                while (assign)
                {
                    assign = false;
                    for (int l = 0; l < m_LinkSize; l++)
                    {
                        if (aryCLink[l].TMC != null && TMCwithRealData.ContainsKey(aryCLink[l].TMC))
                        {
                            temp = 0;
                            tmc_index = TMCwithRealData[aryCLink[l].TMC];
                            for (int d = 0; d < day_size; d++)
                            {
                                temp += mu[d] * (aryCLink[l].Link_Length / TMCHistSpd[tmc_index, d] * 60 - LinkCost[l]);
                            }
                            LinkCost_Penalty[l] = LinkCost[l] + temp;
                            if (LinkCost_Penalty[l] <= 0)  // reduce values of mu in scale
                            {
                                //double mu_sum = 0;
                                for (int d = 0; d < day_size; d++)
                                {
                                    //mu[d] = mu[d] * 0.9;
                                    mu[d] = mu[d] - stepsize_mu[d] + stepsize_mu[d] * 0.9;
                                }
                                //Console.WriteLine(mu_sum);
                                assign = true;
                                break;
                            }
                        }
                        else
                        {
                            Gaussian g = new Gaussian(l);
                            double mean = LinkCost[l];
                            double sd = 0;
                            double g_value = 0;
                            temp = 0;
                            length = aryCLink[l].Link_Length;
                            spd = aryCLink[l].SpeedLimit;
                            for (int d = 0; d < day_size; d++)
                            {
                                sd = Math.Sqrt(day_MeanVar[d]) * (length / spd * 60);
                                g_value = g.NextGaussian3(mean, sd);
                                temp += mu[d] * (g_value - mean);
                            }
                            LinkCost_Penalty[l] = LinkCost[l] + temp;
                            if (LinkCost_Penalty[l] <= 0)  // reduce values of mu in scale
                            {
                                for (int d = 0; d < day_size; d++)
                                {
                                    //mu[d] = mu[d] * 0.9;
                                    mu[d] = mu[d] - stepsize_mu[d] + stepsize_mu[d] * 0.9;
                                }
                                assign = true;
                                break;
                            }
                        }
                    }
                }
                //Console.WriteLine(iteration + "==================");
                // do shortest path search, find path, solve Lx
                if (ShortestPath_Destination(destination_index, out PredAry))
                    GetRoute(true, origin_index, destination_index, PredAry, 0, false, 0, out route);
                else
                    break;
                // solve Lw
                for (int d = 0; d < day_size; d++)
                {
                    w[d] = mu[d] * (day_size - 1) / (2 * nu);
                }

                // find y, solve Ly
                if (beta * Math.Sqrt(y_max) - nu * y_max > 0)
                    y = 0;
                else
                    y = y_max;
                // calculate primal and dual function value
                for (int l = 0; l < route.LinkSeq.Length; l++)
                {
                    average_cost += LinkCost[route.LinkSeq[l]];
                    // calculate travel time for each sample
                    if (aryCLink[route.LinkSeq[l]].TMC != null && TMCwithRealData.ContainsKey(aryCLink[route.LinkSeq[l]].TMC))
                    {
                        tmc_index = TMCwithRealData[aryCLink[route.LinkSeq[l]].TMC];
                        for (int d = 0; d < day_size; d++)
                        {
                            day_cost[d] += aryCLink[route.LinkSeq[l]].Link_Length / TMCHistSpd[tmc_index, d] * 60;
                        }
                    }
                    else
                    {
                        for (int d = 0; d < day_size; d++)
                        {
                            day_cost[d] += aryCLink[route.LinkSeq[l]].TravelTime;
                        }
                    }
                    dual += LinkCost_Penalty[route.LinkSeq[l]];
                    str_route += aryCLink[route.LinkSeq[l]].ID.ID.ToString() + ",";
                }
                for (int d = 0; d < day_size; d++)
                {
                    total_var += Math.Pow(day_cost[d] - average_cost, 2);
                    total_w += (nu / (day_size - 1) * w[d] - mu[d]) * w[d];
                }
                primal = average_cost + beta * Math.Sqrt(total_var / (day_size - 1));
                dual += total_w + beta * Math.Sqrt(y) - nu * y;
                ary_dual[iteration] = dual;

                if (dual > UB)
                    UB = dual;
                if (gap < 0)
                    gap = 0;

                // find optimal
                if (dual >= opt_dual)
                {
                    opt_dual = dual;
                }
                if (primal <= opt_primal)
                {
                    opt_primal = primal;
                    UB = primal;
                }
                gap = opt_primal - opt_dual;
                if (gap <= opt_gap)
                {
                    opt_gap = gap;
                    opt_str_route = str_route;
                }

                // calculate mu(k+1) and nu(k+1) with subgradient
                temp = 0; total_w = 0;
                for (int d = 0; d < day_size; d++)
                {
                    temp = (day_cost[d] - average_cost - w[d]);
                    temp = Math.Abs(temp);
                    if (temp == 0)
                        temp = 0.0001;
                    stepsize_mu[d] = lambda_mu[d] * (UB - dual) / temp;
                    lambda_mu[d] = 1 / (1 / lambda_mu[d] + 1);
                    if (stepsize_mu[d] == 0)
                    {
                        lambda_mu[d] = 0.01;
                        stepsize_mu[d] = lambda_mu[d];
                    }
                    total_w += Math.Pow(w[d], 2);
                }
                temp = total_w / (day_size - 1) - y;
                if (temp == 0)
                    temp = 0.0000001;
                stepsize_nu = lambda_nu * (UB - dual) / temp;
                lambda_nu = 1 / (1 / lambda_nu + 1);
                if (stepsize_nu == 0)
                {
                    lambda_nu = 0.01;
                    stepsize_nu = lambda_nu;
                }
                for (int d = 0; d < day_size; d++)
                {
                    mu[d] += stepsize_mu[d];
                    if (mu[d] < 0)
                        mu[d] = 0;
                }

                nu += stepsize_nu;
                if (nu < 0)
                    nu = 0.0001;

                iteration++;
                log.Write(iteration + "\t" + Math.Round(primal, 4) + "\t" + Math.Round(dual, 4) + "\t" + Math.Round(1 - opt_dual / opt_primal, 4)
                    + "\t" + Math.Round(route.PathTime, 4) + "\n");
            }
            while (iteration < max_iteration);

            log.Close();
            return opt_primal;
        }


    }


}
