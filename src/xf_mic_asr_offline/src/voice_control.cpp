#include "voice_control.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

/************************************************
Function: Example Initialize recording parameters
功能: 初始化录音参数
*************************************************/
int SpeechProcess::record_params_init(record_handle_t* pcm_handle,record_params_t* params)
{
	int err;

	if (pcm_handle == NULL)
	{
		return -1;
	}

	if ((err = snd_pcm_open(&(pcm_handle->pcm),RECORD_DEVICE_NAME,SND_PCM_STREAM_CAPTURE,0))< 0)
	{
		cout << "无法打开音频设备:" << RECORD_DEVICE_NAME << "("<< snd_strerror (err) <<")"<<endl;
		exit(1);
	}

	/*参数结构体，可用于指定PCM流的配置*/
	snd_pcm_hw_params_t *hwparams;

	/*分配硬件参数结构对象，并判断是否分配成功*/
	snd_pcm_hw_params_alloca(&hwparams);

	/*对硬件对象进行初始化默认设置*/
    if((err = snd_pcm_hw_params_any(pcm_handle->pcm,hwparams)) < 0)
    {
    	cout << "初始化参数结构失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

 	/*
    	设置数据为交叉模式(降噪板默认输出单通道PCM)
    	INTERLEAVED/NONINTERLEAVED:交叉/非交叉模式。
    	表示在多声道数据传输的过程中是采样交叉的模式还是非交叉的模式。
    	对多声道数据，如果采样交叉模式，使用一块buffer即可，其中各声道的数据交叉传输；
	如果使用非交叉模式，需要为各声道分别分配一个buffer，各声道数据分别传输。
	*/
    if ((err = snd_pcm_hw_params_set_access(pcm_handle->pcm,hwparams,SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
    	cout << "访问类型设置失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*获取格式设置，并设置pcm数据格式*/
    pcm_handle->format = get_formattype_from_params(params);
    if ((err = snd_pcm_hw_params_set_format(pcm_handle->pcm,hwparams,pcm_handle->format)) < 0)
    {
    	cout << "设置PCM数据格式失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*获取声道并设置*/
    if ((err = snd_pcm_hw_params_set_channels(pcm_handle->pcm,hwparams,params->channel)) < 0)
    {
    	cout << "channel设置失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*获取采样率并设置*/
    if ((err = snd_pcm_hw_params_set_rate_near(pcm_handle->pcm,hwparams, &(pcm_handle->rate),0)) < 0)
    {
    	cout << "采样率设置失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }


    /*将配置写入驱动程序*/
    if ((err = snd_pcm_hw_params(pcm_handle->pcm,hwparams)) < 0)
    {
    	cout << "写入驱动程序设置参数失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*准备音频接口*/
    if ((err = snd_pcm_prepare(pcm_handle->pcm)) < 0)
    {
    	cout << "无法使用音频接口:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

	/*配置一个数据缓冲区用来缓冲数据*/
	pcm_handle->chunk_size = buffer_frames;
    pcm_handle->bits_per_sample = snd_pcm_format_width(pcm_handle->format)/8;
    pcm_handle->bits_per_frame = pcm_handle->bits_per_sample*params->channel;
    pcm_handle->chunk_bytes = pcm_handle->chunk_size*pcm_handle->bits_per_frame;
    pcm_handle->buffer = (unsigned char *)malloc(pcm_handle->chunk_bytes);
    if (!pcm_handle->buffer)
    {
    	cout << "Error malloc" <<endl;
        goto Init_fail;
    }
    // cout << "已初始化录音参数" <<endl;
    return 0;

Init_fail:
	snd_pcm_close(pcm_handle->pcm);
    return -1;
}

/***********************************************
Function: Initialize offline resource parameters
功能: 初始化离线资源参数
************************************************/
int SpeechProcess::init_asr_params(){
	init_rec = 0;
	init_success = 0;
	write_first_data = 0;

	// Build every path with std::string; never write through a const_cast'd
	// c_str(). The vendor ASR API takes writable char*: hand it owned copies.
	std::string root(source_path);
	std::string jet_path   = std::string(begin_) + root + ASR_RES_PATH;
	std::string gram_path  = root + GRM_BUILD_PATH;
	std::string bnf_path   = root + GRM_FILE;
	std::string denoise    = root + DENOISE_SOUND_PATH;

	std::vector<char> jet_buf(jet_path.begin(), jet_path.end());
	jet_buf.push_back('\0');
	std::vector<char> gram_buf(gram_path.begin(), gram_path.end());
	gram_buf.push_back('\0');
	std::vector<char> bnf_buf(bnf_path.begin(), bnf_path.end());
	bnf_buf.push_back('\0');

	// denoise_sound_path is a module-level char* used later by get_record_sound().
	if (denoise_sound_path != NULL) {
		delete[] denoise_sound_path;
	}
	denoise_sound_path = new char[denoise.size() + 1];
	std::copy(denoise.begin(), denoise.end(), denoise_sound_path);
	denoise_sound_path[denoise.size()] = '\0';
	std::cout <<">>>>>denoise_sound_path :" << denoise_sound_path <<std::endl;

	APPID = const_cast<char *>(appid.c_str());

	Recognise_Result inital = initial_asr_paramers(jet_buf.data(), gram_buf.data(), bnf_buf.data(), LEX_NAME);
	if (!inital.whether_recognised)
	{
		std::cout <<"fail_reason :" << inital.fail_reason << std::endl;
		return -1;
	}
	return 0;
}

/**************************************
Function: Get file size
功能: 获取文件大小
***************************************/
int SpeechProcess::filesize(const char *fname)
{
	struct stat statbuf;
    if (stat(fname, &statbuf) == 0)
        return statbuf.st_size;
    return -1;
}

/**************************************
Function: Text encoding conversion
功能: 文本编码转换
***************************************/

std::string SpeechProcess::s2s(const std::string &str)
{
	using convert_typeX =  std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;
	std::wstring wstr = converterX.from_bytes(str);
	return converterX.to_bytes(wstr);
}

/**************************************
Function: Audio format selection
功能: 音频格式选择
***************************************/
snd_pcm_format_t SpeechProcess::get_formattype_from_params(record_params_t* params)
{
    if(params!=NULL){
        switch (params->format) {
        case 0:
            return SND_PCM_FORMAT_S8;
        case 1:
            return SND_PCM_FORMAT_U8;
        case 2:
            return SND_PCM_FORMAT_S16_LE;
        case 3:
            return SND_PCM_FORMAT_S16_BE;
        default:  return SND_PCM_FORMAT_S16_LE;
        }
    }
    return SND_PCM_FORMAT_S16_LE;
}

/**************************************
Function: Text encoding conversion
功能: 送入音频进行识别
***************************************/
void SpeechProcess::business_data_t(unsigned char* record)
{
    record_data = record;
    if (!init_success && init_rec)
    {
        // int len = 3*PCM_MSG_LEN;
        int len = PCM_MSG_LEN;
        std::unique_ptr<char[]> pcm_buffer(new char[len]);
        if (!pcm_buffer)
            {
            	std::cout <<">>>>>buffer is null" <<std::endl;
                return;
            }
        std::memcpy(pcm_buffer.get(), record_data, len);

        if (write_first_data++ == 0)
        {
#if whether_print_log
        	std::cout <<"***************write the first voice**********" <<std::endl;
#endif
            demo_xf_mic(pcm_buffer.get(), len, 1);
        }
        else
        {
#if whether_print_log
        	std::cout <<"***************write the middle voice**********" <<std::endl;
#endif
            demo_xf_mic(pcm_buffer.get(), len, 2);
        }
        if (whether_finised)
        {
            record_finish = 1;
            whether_finised = 0;
        }
    }
}

/**************************************
Function: Get audio data
功能: 获取音频
***************************************/
void SpeechProcess::get_record_sound(const char *fname)
{
	int ret;
	FILE *pcm_file = NULL;
	const char *filename = fname;
    ros_robot_controller_msgs::msg::BuzzerState buzzer;
    buzzer.freq = 3000;
    buzzer.on_time = 0.05;
    buzzer.off_time = 0.01;
    buzzer.repeat = 1;
    buzzer_pub->publish(buzzer);
    sleep(1);
	if ((pcm_file = fopen(filename,"a")) == NULL)
	{
		cout << "无法创建音频文件" <<endl;
		exit(1);
	}
	init_success = record_params_init(&record, &params);
	if (init_success != RET_SUCCESS)
	{
		cout << "音频初始化失败!" <<endl;
		exit(1);
	}
	
    cout<<endl;
	cout<<">>>>>开始一次语音识别！"<<endl;

    while(init_success == RET_SUCCESS){
        if ((ret = snd_pcm_readi(record.pcm,record.buffer,record.chunk_size)) != buffer_frames)
        {
            cout << "从音频接口读取失败:" << "("<< snd_strerror (ret) <<")"<<endl;
            break;
        }
        else if (ret > 0)
        {
            init_rec = 1;
            if (save_pcm_local)
            {
                // Rotate the PCM log safely: close the handle before removing the
                // file, otherwise writes continue into a deleted inode.
                if (filesize(filename) > max_pcm_size)
                {
                    fclose(pcm_file);
                    remove(filename);
                    pcm_file = fopen(filename, "a");
                    if (pcm_file == NULL)
                    {
                        cout << "无法重新创建音频文件" <<endl;
                        exit(1);
                    }
                }
                fwrite(record.buffer, record.chunk_size * params.channel, record.bits_per_sample, pcm_file);
            }
            business_data_t(record.buffer);
        }

        if(record_finish) break;
    }
	init_rec = 0;
	fclose(pcm_file);
	finish_record_sound();
}

/**************************************
Function: Finish recording audio
功能: 结束录制音频
***************************************/
void SpeechProcess::finish_record_sound()
{
	if(record.buffer != NULL) free(record.buffer);
    if(!init_success) snd_pcm_close(record.pcm);
    printf(">>>>>停止录音........\n");
}

/**************************************
Function: Recognition text processing
功能: 识别结果文本处理
***************************************/
Effective_Result SpeechProcess::show_result(char *str)
{
	Effective_Result current;
	current.effective_confidence = 0;
	current.effective_word[0] = '\0';
	if (str == NULL || strlen(str) <= 250)
	{
		std::strncpy(current.effective_word, " ", sizeof(current.effective_word) - 1);
		current.effective_word[sizeof(current.effective_word) - 1] = '\0';
		return current;
	}

	// Parse <focus>...</focus> and <confidence>...</confidence> markers with
	// bounds checks instead of the previous unbounded fixed-size strncpy.
	std::string text(str);
	const std::string focus_start = "<focus>";
	const std::string focus_end = "</focus>";
	const std::string conf_start = "<confidence>";
	const std::string conf_end = "</confidence>";

	std::string word;
	int confidence_int = 0;
	std::size_t fs = text.find(focus_start);
	std::size_t fe = text.find(focus_end, fs == std::string::npos ? 0 : fs);
	std::size_t cs = text.find(conf_start);
	std::size_t ce = text.find(conf_end, cs == std::string::npos ? 0 : cs);

	if (fs != std::string::npos && fe != std::string::npos && fe > fs + focus_start.size())
	{
		word = text.substr(fs + focus_start.size(), fe - (fs + focus_start.size()));
	}
	if (cs != std::string::npos && ce != std::string::npos && ce > cs + conf_start.size())
	{
		std::string conf_text = text.substr(cs + conf_start.size(), ce - (cs + conf_start.size()));
		confidence_int = std::atoi(conf_text.c_str());
	}

	current.effective_confidence = confidence_int;
	if (confidence_int >= confidence)
	{
		std::replace(word.begin(), word.end(), str_, str_none);
		std::strncpy(current.effective_word, word.c_str(), sizeof(current.effective_word) - 1);
		current.effective_word[sizeof(current.effective_word) - 1] = '\0';
	}
	return current;
}

/********************************************************
Function: Get the offline command word recognition result
功能: 获取离线命令词识别结果
*********************************************************/
bool SpeechProcess::Get_Offline_Recognise_Result(const std::shared_ptr<xf_mic_asr_offline_msgs::srv::GetOfflineResult::Request>& request,
							std::shared_ptr<xf_mic_asr_offline_msgs::srv::GetOfflineResult::Response>& response){
	if (request->offline_recognise_start)
	{
		whether_finised = 0;
		record_finish = 0;
		time_per_order = request->time_per_order;
		confidence = request->confidence_threshold;
		int ret = create_asr_engine(&asr_data);
		if (MSP_SUCCESS != ret)
		{
			cout<<"创建语音识别引擎失败！"<<endl;
			return false;
		}

        get_record_sound(denoise_sound_path);
        if(whole_result != "")
		{
			Effective_Result effective_ans = show_result(whole_result);
			if (effective_ans.effective_confidence >= confidence)
			{
				cout<<">>>>>是否识别成功: 是 " <<endl;
				cout<<">>>>>关键字的置信度: [" << effective_ans.effective_confidence << "] " <<endl;
				cout<<">>>>>关键字识别结果: [" << effective_ans.effective_word << "] " <<endl;

				response->result = "ok";
				response->fail_reason = "";
				std::string txt_uft8 = s2s(effective_ans.effective_word);
				response->text = txt_uft8;

				std_msgs::msg::String msg;
				msg.data = effective_ans.effective_word;
				voice_words_pub->publish(msg);
			}
			else
			{
				cout<<">>>>>是否识别成功: 否 " <<endl;
				cout<<">>>>>关键字的置信度: [" << effective_ans.effective_confidence << "] " <<endl;
				cout<<">>>>>关键字置信度较低，文本不予显示" <<endl;

				response->result = "fail";
				response->fail_reason = "low_confidence error or 11212_license_expired_error";
				response->text = " ";
			}
		}
		else
		{
			response->result = "fail";
			response->fail_reason = "no_valid_sound error";
			response->text = " ";
			cout<<">>>>>未能检测到有效声音,请重试" <<endl;
		}
		
        whole_result = "";
		/*[1-3]语音识别结束]*/
		delete_asr_engine();
		write_first_data = 0;
		sleep(1.0);
	}
	cout<<endl;
	return true;
}

SpeechProcess::SpeechProcess(const std::string &node_name)
: rclcpp::Node(node_name){
	/***声明参数并获取***/
	this->declare_parameter<string>("appid","");
	this->declare_parameter<string>("source_path","");
	this->get_parameter("appid",appid);
	cout << "appid:" << appid << endl;
    this->get_parameter("source_path",source_path);

	voice_words_pub = this->create_publisher<std_msgs::msg::String>("~/voice_words",10);
    buzzer_pub = this->create_publisher<ros_robot_controller_msgs::msg::BuzzerState>("/ros_robot_controller/set_buzzer", 10);
	get_offline_result_srv_ = this->create_service<xf_mic_asr_offline_msgs::srv::GetOfflineResult>(
		"~/get_offline_result",[this](const std::shared_ptr<xf_mic_asr_offline_msgs::srv::GetOfflineResult::Request> request,
									std::shared_ptr<xf_mic_asr_offline_msgs::srv::GetOfflineResult::Response> response){
									Get_Offline_Recognise_Result(request,response);
		});

	int ret = init_asr_params();
	if(ret == RET_SUCCESS)
	{
		RCLCPP_INFO(this->get_logger(),"Initialization Offline resource parameter success!");
	}

	cal_task_ = std::thread(&SpeechProcess::run, this);
}

/********************************************************
Function: Calculates whether recording times out
功能: 检测录音是否超时
*********************************************************/
void SpeechProcess::run()
{
	rclcpp::Time start_time,last_time;
	while(rclcpp::ok()){
		if (init_rec){
			start_time = rclcpp::Node::now();
            while(init_rec && whether_finised != 1){
				last_time = rclcpp::Node::now();
				if ((last_time - start_time).seconds() > time_per_order){
					std::cout <<">>>>>超出离线命令词最长识别时间" << std::endl;
					whether_finised = 1;
					break;
				}
			}
		}
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
	}
}

SpeechProcess::~SpeechProcess()
{
	record_finish = 1;
	if (cal_task_.joinable())
	{
		cal_task_.join();
	}
	RCLCPP_INFO(this->get_logger(),"voice_control node over!\n");
}

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<SpeechProcess>("voice_control"));
  	rclcpp::shutdown();
	return 0;
}
