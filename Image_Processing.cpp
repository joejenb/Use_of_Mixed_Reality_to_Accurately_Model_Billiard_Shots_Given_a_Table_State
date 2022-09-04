#include "Image_Processing.hpp"

static void detect_edges(cv::Mat &col_frame, cv::Mat &depth_frame, cv::Mat &vid_edges)
{
	cv::Mat dgrey_frame, cgrey_frame, blended, bi_filter, thresh_frame;
	cv::cvtColor(col_frame, cgrey_frame, cv::COLOR_BGR2GRAY);
	cv::threshold(depth_frame, thresh_frame, 50, 255, cv::THRESH_BINARY);
	cv::bitwise_and(cgrey_frame, thresh_frame, blended);
	cv::imshow("masked", blended);
	cv::blur(blended, bi_filter, cv::Size(5, 5));
	cv::Canny(bi_filter, vid_edges, 120, 190, 5);
}

static void extract_lines(cv::Mat edge_frame, std::vector<cv::Vec2f> &hor_lines, std::vector<cv::Vec2f> &ver_lines)
{
	std::vector<cv::Vec2f> frame_lines;
	double drad = CV_PI / 180;
					//image,  lines,	   rho, theta, threshold, srn, stn
	cv::HoughLines(edge_frame, frame_lines, 1, drad, 90);

	for (auto &line: frame_lines)
	{
		float rad = line[0], angle = line[1];
		if ((89 * drad < angle) && (angle < 91 * drad))
		{
			hor_lines.push_back(line);
		}
		else if ((-1 * drad < angle) && (angle < 1 * drad))
		{
			ver_lines.push_back(line);
		}
	}
}

static void calculate_corners(std::vector<cv::Vec2f> &hor_lines, std::vector<cv::Vec2f> &ver_lines, std::vector<cv::Point>& corners, cv::Point &vid_centre)
{
	double deter;
	cv::Point up_r, up_l, bot_l, bot_r, point1, point2, intersection;
	up_r.x = bot_r.y = bot_r.x = bot_l.y = vid_centre.x * 2000;
	up_r.y = bot_l.x = up_l.x = up_l.y = 0;

	for (auto& line1 : hor_lines)
	{
		double radi1 = line1[0], angle1 = line1[1];
		point1.x = radi1 * cos(angle1); point1.y = radi1 * sin(angle1);
		for (auto& line2 : ver_lines)
		{
			double radi2 = line2[0], angle2 = line2[1];
			point2.x = radi2 * cos(angle2); point2.y = radi2 * sin(angle2);
			deter = (cos(angle1) * sin(angle2)) - (cos(angle2) * sin(angle1));
			intersection.x = ((radi1 * sin(angle2)) - (radi2 * sin(angle1))) / deter;
			intersection.y = ((radi2 * cos(angle1)) - (radi1 * cos(angle2))) / deter;
			//split into quadrant, if hypotenuse is smaller than that of current corner -> choose instead 
			//Must be in right side
			if (intersection.x > vid_centre.x)
			{
				//Must be in bottom right
				if (intersection.y > vid_centre.y)
				{
					if ((pow((intersection.y - vid_centre.y), 2) + pow((intersection.x - vid_centre.x), 2)) < (pow((bot_r.y - vid_centre.y), 2) + pow((bot_r.x - vid_centre.x), 2)))
					{
						bot_r = intersection;
					}
				}
				//Must be in top right
				else 
				{
					if ((pow((vid_centre.y - intersection.y), 2) + pow((intersection.x - vid_centre.x), 2)) < (pow((vid_centre.y - up_r.y), 2) + pow((up_r.x - vid_centre.x), 2)))
					{
						up_r = intersection;
					}
				}
			}
			else
			{
				if (intersection.y > vid_centre.y)
				{
					if ((pow((intersection.y - vid_centre.y), 2) + pow((vid_centre.x - intersection.x), 2)) < (pow((bot_l.y - vid_centre.y), 2) + pow((vid_centre.x - bot_l.x), 2)))
					{
						bot_l = intersection;
					}
				}
				else 
				{
					if ((pow((vid_centre.y - intersection.y), 2) + pow((vid_centre.x - intersection.x), 2)) < (pow((vid_centre.y - up_l.y), 2) + pow((vid_centre.x - up_l.x), 2)))
					{
						up_l = intersection;
					}
				}
			}
		}
	}
	corners = { up_l, up_r, bot_l, bot_r };
}

static void localise_table(cv::Mat &colour_frame, cv::Mat &depth_frame, std::vector<cv::Point> &corners)
{
	std::vector<cv::Vec2f> hor_lines, ver_lines; 
	cv::Mat out_frame;
	cv::Mat vid_edges;
	cv::Point vid_centre;
	vid_centre.x = colour_frame.cols / 2;
	vid_centre.y = colour_frame.rows / 2;
	
	detect_edges(colour_frame, depth_frame, vid_edges);
	cvtColor(vid_edges, out_frame, cv::COLOR_GRAY2BGR);
	extract_lines(vid_edges, hor_lines, ver_lines);
	calculate_corners(hor_lines, ver_lines, corners, vid_centre);
	for (auto& inter : corners)
	{
		cv::line(out_frame, cv::Point(inter.x+15, inter.y), cv::Point(inter.x-15, inter.y), cv::Scalar(0, 255, 0), 3);
		cv::line(out_frame, cv::Point(inter.x, inter.y+15), cv::Point(inter.x, inter.y-15), cv::Scalar(0, 255, 0), 3);
	}
	cv::circle(out_frame, vid_centre, 2, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	cv::imshow("corners", out_frame);
}

static void identify_balls(cv::Mat colour_frame, int radius, std::vector<cv::Point2f> &centres, std::vector<cv::Scalar> &ball_colours, int &white_ball)
{
	std::vector<cv::Mat> planes;
	cv::split(colour_frame, planes);
	cv::MatND hist;
	int hist_size = 100;
	float range[] = { 0, 256 };
	int bin_w = std::round((double)256 / hist_size);
	const float* hist_range = { range };
	/*Want to get loaction of cue ball -> if intersect draw line from cue ball center -> angle of cue is then mirrored to get rebound line
	if line then also intersects with other ball -> where? */
	int max_col = 0;
	for (auto ball : centres)
	{
		/*redo without histogram*/
		cv::Mat b_hist, g_hist, r_hist;
		double b_min, b_max, g_min, g_max, r_min, r_max;
		cv::Point b_min_loc, b_max_loc, g_min_loc, g_max_loc, r_min_loc, r_max_loc;
		cv::Mat roi_mask = cv::Mat::zeros(colour_frame.size(), CV_8UC1); 
		cv::circle(roi_mask, ball, radius, cv::Scalar(255), cv::FILLED, cv::LINE_8);
		cv::calcHist(&planes[0], 1, 0, roi_mask, b_hist, 1, &hist_size, &hist_range, true, false);
		cv::calcHist(&planes[1], 1, 0, roi_mask, g_hist, 1, &hist_size, &hist_range, true, false);
		cv::calcHist(&planes[2], 1, 0, roi_mask, r_hist, 1, &hist_size, &hist_range, true, false);
		cv::minMaxLoc(b_hist, &b_min, &b_max, &b_min_loc, &b_max_loc);
		cv::minMaxLoc(g_hist, &g_min, &g_max, &g_min_loc, &g_max_loc);
		cv::minMaxLoc(r_hist, &r_min, &r_max, &r_min_loc, &r_max_loc);
		cv::Scalar ball_col = { (float)b_max_loc.y * bin_w, (float)g_max_loc.y * bin_w, (float)r_max_loc.y * bin_w };
		if (ball_col[0] + ball_col[1] + ball_col[2] > max_col)
		{
			max_col = ball_col[0] + ball_col[1] + ball_col[2];
			white_ball = ball_colours.size();
		}
		ball_colours.push_back(ball_col);

		cv::circle(colour_frame, ball, radius, {(float)b_max_loc.y*bin_w, (float)g_max_loc.y*bin_w, (float)r_max_loc.y*bin_w}, 2, cv::LINE_8);
	}
	cv::imshow("balls identified", colour_frame);
}

float localise_balls(cv::Mat &depth_frame, cv::Mat &avg_depth_frame, std::vector<cv::Point2f> &centres)
{
	/*due to table being potentialy unlevel and a inaccuracy in finding target area, subtract average transformed depth frame to get just the balls
	-> could also do with just a standard camera*/
	/*Need to threshold according to depth -> requires knowing ball height -> subtract original table from photo with balls
	-> to get height in pixel value -> then sample from original for values greater than 6/8 x the ball width -> do this
	once have realsense camera itself*/
	/*Then apply morphological erosion operator to thresholded image to remove small blobs and noise -> Apply contour finder */

	cv::Mat thresh_frame, edge_frame, diff_frame;
	cv::Mat struct_el = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10), cv::Point(0, 0));
	cv::absdiff(depth_frame, avg_depth_frame, diff_frame);
	cv::threshold(diff_frame, thresh_frame, 30, 255, cv::THRESH_BINARY);
	
	cv::erode(thresh_frame, thresh_frame, struct_el);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(thresh_frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<cv::Point> > contours_poly(contours.size());
	std::vector<float> radi(contours.size());
	centres.resize(contours.size());

	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
		cv::minEnclosingCircle(contours_poly[i], centres[i], radi[i]);
		cv::circle(depth_frame, centres[i], radi[i], cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	}
	cv::imshow("balls", depth_frame);
	if (radi.size())
	{
		return *std::max_element(radi.begin(), radi.end());
	}
	return 0;
}

void average_image(Video_Feed &depth_feed,  cv::Mat& avg_col, cv::Mat& avg_depth)
{
	//Wrong width and height
	cv::Mat col_accumulator, depth_accumulator;
	int num_of_frames = 0;
	while (num_of_frames < 10)
	{
		cv::Mat col_image;
		if (depth_feed.get_colour_frame(col_image))
		{
			if (!num_of_frames)
			{
				col_accumulator = cv::Mat(col_image.size(), CV_64FC3);
				col_accumulator.setTo(cv::Scalar(0, 0, 0, 0));
			}
			col_accumulator += col_image;
			num_of_frames++;
		}
	}
	num_of_frames = 0;
	while (num_of_frames < 10)
	{
		cv::Mat depth_image;
		if (depth_feed.get_depth_frame(depth_image))
		{
			if (!num_of_frames)
			{
				depth_accumulator = cv::Mat(depth_image.size(), CV_64FC1);
				depth_accumulator.setTo(cv::Scalar(0));
			}
			depth_accumulator += depth_image;
			num_of_frames++;
		}
	}
	col_accumulator.convertTo(avg_col, CV_8UC3, 1. / 10);
	depth_accumulator.convertTo(avg_depth, CV_8UC1, 1. / 10);
}

Video_Feed::Video_Feed() :
	alive(true),
	transform_matrix(3, 3, CV_8UC1, cv::Scalar(0)),
	k(0),
	motion(false),
	previous_motion(false)
{
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8);
	auto profile = pipe.start(cfg);
	auto sensor = profile.get_device();
	auto depth_sensor = sensor.first<rs2::depth_sensor>();
	auto advanced_sensor = sensor.as<rs400::advanced_mode>();

	depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_CUSTOM);
	if (!advanced_sensor.is_enabled())
	{
		// Enable advanced-mode
		advanced_sensor.toggle_advanced_mode(true);
	}

	//Temporal filter is all that's really needed
	STDepthTableControl table = advanced_sensor.get_depth_table();
	table.depthUnits = 900;
	table.disparityShift = 50;
	advanced_sensor.set_depth_table(table); 
}

void Video_Feed::start()
{
	processing_thread = std::thread([&]() 
	{
		rs2::colorizer colour_map(2);
		rs2::threshold_filter thresh_filter;
		rs2::spatial_filter spat_filter;    
		rs2::temporal_filter temp_filter;  
		rs2::align align_to(RS2_STREAM_COLOR);
		rs2::disparity_transform depth_to_disparity(true);
		rs2::disparity_transform disparity_to_depth(false);
		colour_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, false);
		colour_map.set_option(RS2_OPTION_MIN_DISTANCE, 0.7);
		colour_map.set_option(RS2_OPTION_MAX_DISTANCE, 0.9);

		while (alive)
		{
			rs2::frameset frames;
			rs2::frame depth_frame, filtered, colour_frame;
			if (pipe.poll_for_frames(&frames))
			{
				frames = frames.apply_filter(align_to);
				depth_frame = frames.first(RS2_STREAM_DEPTH);
				colour_frame = frames.first(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
				filtered = depth_frame.apply_filter(depth_to_disparity);
				filtered = filtered.apply_filter(disparity_to_depth);
				filtered = filtered.apply_filter(colour_map);
				depth_frames.enqueue(filtered);
				colour_frames.enqueue(colour_frame);
			}
		}
	});
}

void Video_Feed::classify_motion(cv::Mat current_frame)
{
	cv::Mat previous_frame, diff_frame;
	int queue_size = previous_frames.size();

	cv::GaussianBlur(current_frame, current_frame, cv::Size(7, 7), 2, 2);

	cv::Mat difference_accumulator(current_frame.size(), CV_8UC1);
	difference_accumulator.setTo(cv::Scalar(0));

	for (int num_frames = 0; num_frames < queue_size; num_frames++)
	{
		previous_frame = previous_frames.front();
		previous_frames.pop();
		previous_frames.push(previous_frame);
		cv::absdiff(previous_frame, current_frame, diff_frame);
		difference_accumulator += diff_frame;
	}
	previous_frames.push(current_frame);

	//Has now dequeued and enqueued every frame -> in original state
	if (queue_size >= 3)
	{
		previous_frames.pop();
	}

	cv::Mat avg_diff;
	cv::threshold(difference_accumulator, avg_diff, 35, 255, cv::THRESH_BINARY);
	avg_diff.convertTo(avg_diff, CV_8UC1);

	previous_motion = motion;
	int num_moved = cv::countNonZero(avg_diff);
	bool new_motion = num_moved > 0.0005 * avg_diff.rows * avg_diff.cols;
	if (motion ^ new_motion)
	{
		k += 1;
	}
	else
	{
		k -= 1;
	}

	if (k < 0)
	{
		k = 0;
	}

	if (k == 15)
	{
		motion = !motion;
		k = 0;
	}
	to_show = avg_diff;
}

bool Video_Feed::get_depth_frame(cv::Mat& depth_frame)
{
	rs2::frame depth;
	if (depth_frames.poll_for_frame(&depth))
	{
		int width = depth.as<rs2::video_frame>().get_width();
		int height = depth.as<rs2::video_frame>().get_height();
		depth_frame = cv::Mat(cv::Size(width, height), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
		cv::cvtColor(depth_frame, depth_frame, cv::COLOR_BGR2GRAY);
		return true;
	}
	return false;
}


bool Video_Feed::get_colour_frame(cv::Mat& colour_frame)
{
	rs2::frame colour;
	if (colour_frames.poll_for_frame(&colour))
	{
		int width = colour.as<rs2::video_frame>().get_width();
		int height = colour.as<rs2::video_frame>().get_height();
		colour_frame = cv::Mat(cv::Size(width, height), CV_8UC3, (void*)colour.get_data(), cv::Mat::AUTO_STEP);
		return true;
	}
	return false;
}

bool detect_cue(cv::Mat &ball_frame, cv::Mat &current_frame, float ball_size, cv::Mat& biggest_blob, float& max_x, float& max_y, cv::Point& cue_centre)
{
	cv::Mat diff_frame, thresh_frame;
	cv::absdiff(ball_frame, current_frame, diff_frame);
	cv::threshold(diff_frame, thresh_frame, 10, 255, cv::THRESH_BINARY);

	cv::Mat struct_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
	cv::erode(thresh_frame, biggest_blob, struct_el, cv::Point(-1, -1), 2);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(biggest_blob, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<float> radi(contours.size());
	//Could identify points on edge of image -> gives dierction of cue -> adjust appropriately
	//Line must always point away from the cue centre -> so for x and y do difference between ball centre and cue centre -> normalise -> multiply by 
	//delts
	for (size_t i = 0; i < contours.size(); i++)
	{
		double contour_area = cv::contourArea(contours[i]);
		if (contour_area > 2 * CV_PI * ball_size * ball_size)
		{
			cv::RotatedRect bounded_cue = cv::minAreaRect(contours[i]);
			cue_centre = bounded_cue.center;
			cv::Point2f rect_points[4];
			bounded_cue.points(rect_points);

			for (int i = 0; i < 4; ++i)
			{
				if (rect_points[i].x <= 0 || rect_points[i].x >= current_frame.cols || rect_points[i].y <= 0 || rect_points[i].y >= current_frame.rows)
				{
					float delt_x1 = -rect_points[i].x + rect_points[(i + 1) % 4].x;
					float delt_y1 = -rect_points[i].y + rect_points[(i + 1) % 4].y;
					float delt_x2 = -rect_points[i].x + rect_points[(i - 1) % 4].x;
					float delt_y2 = -rect_points[i].y + rect_points[(i - 1) % 4].y;
					max_x = delt_x1 * delt_x1 + delt_y1 * delt_y1 > delt_x2 * delt_x2 + delt_y2 * delt_y2 ? delt_x1 : delt_x2;
					max_y = delt_x1 * delt_x1 + delt_y1 * delt_y1 > delt_x2 * delt_x2 + delt_y2 * delt_y2 ? delt_y1 : delt_y2;
					return true;
				}
			}
		}
	}
	return false;
}

void rescale(cv::Point start, cv::Point &aim, float max_x, float max_y)
{
	float ratio;
	if (aim.x > max_x)
	{ 
		ratio = ((float)max_x - (float)start.x) / ((float)aim.x - (float)start.x);
		aim.x = max_x;
		aim.y = start.y + cvRound((aim.y - start.y) * ratio);
	}
	else if (aim.x < 0)
	{
		ratio = (0.0 - (float)start.x) / ((float)aim.x - (float)start.x);
		aim.x = 0;
		aim.y = start.y + cvRound((aim.y - start.y) * ratio);
	}
	else if (aim.y > max_y)
	{
		ratio = ((float)max_y - (float)start.y) / ((float)aim.y - (float)start.y);
		aim.y = max_y;
		aim.x = start.x + cvRound((aim.x - start.x) * ratio);
	}
	else if (aim.y < 0)
	{
		ratio = (0.0 - (float)start.y) / ((float)aim.y - (float)start.y);
		aim.y = 0;
		aim.x = start.x + cvRound((aim.x - start.x) * ratio);
	}
}


void calculate_trajectory(float delt_x, float delt_y, cv::Point cue_centre, std::vector<cv::Point2f>& centres, float ball_radi, int white_ball, cv::Mat &tmp_image, Shot_Data &shot_data)
{
	/*Create two mats of zeros -> draw only white ball on one -> get point of intersection with cue -> if one the must be another on other side
	-> skip first identify second -> start from point inside the image i.e the cue tip -> draw line from second intersection -> calculate 
	perpendicuar line extend by ball_radi from centre of white ball -> draw new lines from these points -> */
	cv::Mat cue_ball = cv::Mat::zeros(tmp_image.size(), CV_8UC1);
	cv::Mat object_ball = cv::Mat::zeros(tmp_image.size(), CV_8UC1);
	bool strike = false;
	cv::circle(cue_ball, centres[white_ball], ball_radi, cv::Scalar(255), 1, cv::LINE_8);
	for (int ball = 0; ball < centres.size(); ball++)
	{
		if (ball != white_ball)
			cv::circle(object_ball, centres[ball], ball_radi, cv::Scalar(255), 1, cv::LINE_8);
	}
	cv::Point aim(cvRound(cue_centre.x + 1000 * delt_x), cvRound(cue_centre.y + 1000 * delt_y));
	rescale(cue_centre, aim, tmp_image.cols, tmp_image.rows);

	cv::Point contact;
	cv::Point centre_atc;
	cv::Point resultant;

	float hyp = std::sqrt(delt_x * delt_x + delt_y * delt_y);
	float shift_x = delt_x / (hyp / ball_radi);
	float shift_y = delt_y / (hyp / ball_radi);
	cv::LineIterator cue_line(cue_ball, cue_centre, aim);
	cv::LineIterator object_line(cue_ball, cue_centre, aim);
	for (int cuep = 0; (cuep < cue_line.count) && !strike; cuep++, ++cue_line)
	{
		if (cue_ball.at<uchar>(cue_line.pos()) != 0)
		{
			/*Calculate angle of line -> get that perpendicular -> length of hypotenuse is the ball radi -> adjust x and y's for this*/
			//transformed lines are wrong here

			object_line = cv::LineIterator(object_ball, centres[white_ball], cv::Point(centres[white_ball].x + 1000 * delt_x, centres[white_ball].y + 1000 * delt_y));
			strike = true;
		}
	}

	if (!strike)
	{
		cv::line(tmp_image, cue_centre, aim, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
		shot_data.cue_x1 = ((float)cue_centre.x / (float)tmp_image.cols) - 0.5;
		shot_data.cue_y1 = 1.0 - ((float)cue_centre.y / (float)tmp_image.rows) - 0.5;
		shot_data.cue_x2 = ((float)aim.x / (float)tmp_image.cols) - 0.5;
		shot_data.cue_y2 = 1.0 - ((float)aim.y / (float)tmp_image.rows) - 0.5;
	}
	else
	{
		cv::Point white_aim(cvRound(centres[white_ball].x + 1000 * delt_x), cvRound(centres[white_ball].y + 1000 * delt_y));
		rescale(centres[white_ball], white_aim, tmp_image.cols, tmp_image.rows);
		cv::line(tmp_image, centres[white_ball], white_aim, cv::Scalar(0, 0, 255));
		shot_data.cue_x1 = ((float)centres[white_ball].x / (float)tmp_image.cols) - 0.5;
		shot_data.cue_y1 = 1.0 - ((float)centres[white_ball].y / (float)tmp_image.rows) - 0.5;
		shot_data.cue_x2 = ((float)white_aim.x / (float)tmp_image.cols) - 0.5;
		shot_data.cue_y2 = 1.0 - ((float)white_aim.y / (float)tmp_image.rows) - 0.5;
		strike = false;
		for (int objp = 0; (objp < object_line.count) && !strike; objp++, ++object_line)
		{
			for (int ball = 0; ball < centres.size(); ball++)
			{
				if (ball != white_ball)
				{
					float diff_x = object_line.pos().x - centres[ball].x;
					float diff_y = object_line.pos().y - centres[ball].y;
					if (std::abs((diff_x * diff_x) + (diff_y * diff_y) - (4 * ball_radi * ball_radi)) <= 150)
					{
						centre_atc = object_line.pos();
						resultant.x = cvRound(centres[ball].x - 1000 * diff_x);
						resultant.y = cvRound(centres[ball].y - 1000 * diff_y);
						strike = true;
					}
				}
			}
		}
	}
	if (strike)
	{
		rescale(centre_atc, resultant, tmp_image.cols, tmp_image.rows);
		cv::line(tmp_image, centre_atc, resultant, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
		shot_data.object_x1 = ((float)centre_atc.x / (float)tmp_image.cols) - 0.5;
		shot_data.object_y1 = 1.0 - ((float)centre_atc.y / (float)tmp_image.rows) - 0.5;
		shot_data.object_x2 = ((float)resultant.x / (float)tmp_image.cols) - 0.5;
		shot_data.object_y2 = 1.0 - ((float)resultant.y / (float)tmp_image.rows) - 0.5;
		cv::circle(tmp_image, centre_atc, ball_radi, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
	}
}

std::string to_json(Shot_Data& shot_data )
 {
     std::string json_obj = "{ \"resultant_x1\": " + std::to_string(shot_data.resultant_x1);
     json_obj += ", \"resultant_y1\": " + std::to_string(shot_data.resultant_y1);
     json_obj += ", \"resultant_x2\": " + std::to_string(shot_data.resultant_x2);
     json_obj += ", \"resultant_y2\": " + std::to_string(shot_data.resultant_y2);
     json_obj += ", \"object_x1\": " + std::to_string(shot_data.object_x1);
     json_obj += ", \"object_y1\": " + std::to_string(shot_data.object_y1);
     json_obj += ", \"object_x2\": " + std::to_string(shot_data.object_x2);
     json_obj += ", \"object_y2\": " + std::to_string(shot_data.object_y2);
     json_obj += ", \"cue_x1\": " + std::to_string(shot_data.cue_x1);
     json_obj += ", \"cue_y1\": " + std::to_string(shot_data.cue_y1);
     json_obj += ", \"cue_x2\": " + std::to_string(shot_data.cue_x2);
     json_obj += ", \"cue_y2\": " + std::to_string(shot_data.cue_y2);
     json_obj += ", \"radi\": " + std::to_string(shot_data.radi) + "}";
     return json_obj;
 }
 
 
void transfer_data(std::deque<Shot_Data> *transfer_q)
{
	WSAData wsaData;
	SOCKADDR_IN addr;

	addr.sin_family = AF_INET;
	addr.sin_port = htons(DEFAULT_PORT);
	addr.sin_addr.s_addr = inet_addr("192.168.1.164");

	//int angle = 0;
	Shot_Data shot_data;// = { 0, 0.25, 0, 0.35, 0, -0.25, 0, -0.35, 0, 0.35, 0, -0.35, 0.08 };
	while (true)
	{
		int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
		SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		result = connect(s, (SOCKADDR*)&addr, sizeof(addr));

		int error_code;
		int error_code_size = sizeof(error_code);
		getsockopt(s, SOL_SOCKET, SO_ERROR, (char*)&error_code, &error_code_size);
		while (!error_code)
		{
			trans_lock.lock();
			if (!transfer_q->empty())
			{
				shot_data = transfer_q->front();
				transfer_q->pop_front();
				int count = 0;
				char sendBuf[1024];
				strcpy(sendBuf, to_json(shot_data).c_str());

				printf("\n%d  %d\n", strlen(sendBuf));
				printf(sendBuf);
				int sendResult;
				sendResult = send(s, sendBuf, strlen(sendBuf), 0);
			}
			trans_lock.unlock();
		}
		shutdown(s, SD_SEND);
	}
}

int main(int argc, char* argv[])// try
{
	cv::Mat trans_d, trans_c, live_feed_d, live_feed_c, ball_frame;
	cv::Mat transform_matrix, inv_transform_matrix, avg_col, avg_depth, trajectories;
	std::vector<cv::Scalar> ball_colours;
	std::vector<cv::Point2f> centres;
	float ball_radi = 0.0;
	int white_ball = 0;
	
	std::deque<Shot_Data> transfer_q;
	std::thread thread_obj(transfer_data, &transfer_q);

	Video_Feed depth_feed;
	depth_feed.start();
	
	average_image(depth_feed, avg_col, avg_depth);
	std::vector<cv::Point> corners, corners_transformed{ cv::Point(0,0), cv::Point(avg_depth.cols,0), cv::Point(0,avg_depth.rows), cv::Point(avg_depth.cols,avg_depth.rows) };

	localise_table(avg_col, avg_depth, corners);
	transform_matrix = cv::findHomography(corners, corners_transformed, cv::RANSAC);
	inv_transform_matrix = cv::findHomography(corners_transformed, corners, cv::RANSAC);
	cv::warpPerspective(avg_depth, avg_depth, transform_matrix, avg_depth.size());
	cv::warpPerspective(avg_col, avg_col, transform_matrix, avg_col.size());
	depth_feed.transform_matrix = transform_matrix;

	cv::Mat roi_mask = cv::Mat::zeros(avg_depth.size(), CV_8U); 
	roi_mask(cv::Rect(20, 20, 20, 20)) = 1;
	cv::Scalar avg_depth_val = cv::mean(avg_depth, roi_mask);

	while (cv::waitKey(1) < 0)
	{
		if (depth_feed.get_depth_frame(live_feed_d) && depth_feed.get_colour_frame(live_feed_c))
		{
			cv::warpPerspective(live_feed_d, trans_d, transform_matrix, live_feed_d.size());
			//cv::imshow("Depth", trans_d);
			cv::warpPerspective(live_feed_c, trans_c, transform_matrix, live_feed_c.size());

			depth_feed.classify_motion(trans_d.clone());
			if (!depth_feed.motion && depth_feed.previous_motion)
			{
				std::cout << "STOPPED" << std::endl;
				white_ball = 0;
				centres.clear();
				ball_colours.clear();
				ball_frame = trans_d.clone();
				float max_found = localise_balls(trans_d, avg_depth, centres);
				identify_balls(trans_c, ball_radi, centres, ball_colours, white_ball);
				if (!ball_radi)
					ball_radi = max_found;
			}
			else if (depth_feed.motion && !depth_feed.previous_motion)
			{
				std::cout << "STARTED" << std::endl;
			}
			else if (!depth_feed.motion && !depth_feed.previous_motion)
			{
				std::cout << "NON EXISTENT" << std::endl;
			}
			else if (depth_feed.motion && depth_feed.previous_motion)
			{
				std::cout << "MOVING" << std::endl;
				cv::Mat biggest_blob(trans_d.size(), CV_8UC1, cv::Scalar(0));
				float delt_x = 0, delt_y = 0;
				cv::Point cue_centre;
				if (!ball_frame.empty() && detect_cue(ball_frame, trans_d, ball_radi, biggest_blob, delt_x, delt_y, cue_centre))
				{
					cv::Mat tmp_image = cv::Mat::zeros(trans_c.size(), CV_8UC3);
					Shot_Data shot_data = { 0 };
					calculate_trajectory(delt_x, delt_y, cue_centre, centres, ball_radi, white_ball, tmp_image, shot_data);
					trans_lock.lock();
					transfer_q.push_back(shot_data);
					trans_lock.unlock();
					cv::warpPerspective(tmp_image, trajectories, inv_transform_matrix, tmp_image.size());
					cv::bitwise_or(live_feed_c, trajectories, live_feed_c);
				}
					
			}
			//cv::imshow("Motion", trans_c);
			cv::imshow("Full Feed", live_feed_c);
		}
	}
	depth_feed.alive = false;
	depth_feed.processing_thread.join();
	thread_obj.join();
	return EXIT_SUCCESS;
}
