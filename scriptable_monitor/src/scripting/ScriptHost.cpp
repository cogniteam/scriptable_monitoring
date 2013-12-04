/*
 * ScriptHost.cpp
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/ScriptHost.h>

ScriptHost::ScriptHost()
{
	_executionInterval = 0.1;
	RosTopicListener::start();
}

ScriptHost::~ScriptHost()
{
	stop();
	RosTopicListener::stop();
}

AddScriptResponse ScriptHost::addScript(string sourceCode)
{
	lock_recursive(_scriptsMutex);
	AddScriptResponse response;
	response.message = "";

	set<string> internalFunctions;
	foreach (string functionName, InternalFunctionsManager::getFunctionNames()) {
		internalFunctions.insert(functionName);
	}

	/**
	 * Create predicate script from the source code
	 */
	PredicatesScript predicateScript(sourceCode, internalFunctions);
	if (predicateScript.getName() == "") {
		cerr << "[e] Unnamed script provided, cannot add to ScriptHost" << endl;
		response.message = "[e] Unnamed script provided, cannot add to ScriptHost";
		response.success = false;
		return response;
	}

	if (scriptExists(predicateScript.getName())) {
		cerr << "[e] Script with the same name already exists" << endl;
		response.message = "[e] Script with the same name already exists";
		response.success = false;
		return response;
	}

	/**
	 * Convert to python script, simulate execution to extract used topic names
	 */
	PythonScript* pythonScript = new PythonScript(predicateScript.getPythonScript());
	bool validScript = prepareScript(*pythonScript);

	if (validScript)
		_scripts.insert(boost::shared_ptr<PythonScript>(pythonScript));
	else {
		cerr << "[e] Invalid script" << endl;
		response.message = "[e] Invalid script";
		response.success = false;
		return response;
	}

	/**
	 * Add used topics to listener
	 */
	foreach (string topicName, pythonScript->getUsedTopics()) {
		RosTopicListener::addTopic(topicName);
	}

	cout << "[+] Script added [" << pythonScript->getName() << "]" << endl;
	response.message = "[+] Script added";
	response.success = true;
	return response;
}

bool ScriptHost::prepareScript(PythonScript& script)
{
	CompilationResult result = _executer.compile(script);
	if (!result.success) {
		cerr << "[e] Compilation of script '" << script.getName() << "' failed" << endl;
		cerr << "[e] Message: " << result.message << endl;
		return false;
	}

	/**
	 * Extracts used topics, validations, internal function etc...
	 */
	_executer.simulate(script);

	return true;
}

void ScriptHost::run()
{
	while (!_workThread->interruption_requested()) {

		{
			lock_recursive(_scriptsMutex);
			foreach (PythonScriptPtr script, _scripts) {

				if (!isExecutionTime(script))
					continue;

				if (!hasAllTopicValues(script))
					continue;

//				cout << "[i] Time to execute '" << script->getName() << "'" << endl;
				_executer.execute(*script, RosTopicListener::getTopicsValues());
				script->updateExecutionTime();

//				cout << "[i] Validations: " << (script->isValidationFailed() ? "FAILED [" + script->getFailedValidation() + "]" : "PASSED") << endl;

				addDiagnosticStatus(script);

			}
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0 * _executionInterval));
	}
}

void ScriptHost::start()
{
	RosTopicListener::start();

	if (!_workThread)
		_workThread = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&ScriptHost::run, this)));
}

void ScriptHost::stop()
{
	ROS_INFO("Stopping ScriptHost...");
	_workThread->interrupt();
	_workThread->join();
}

bool ScriptHost::isExecutionTime(PythonScriptPtr script)
{
	boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime nextExecutionTime
		= script->getExecutionTime() + boost::posix_time::time_duration(boost::posix_time::milliseconds(1000.0 * script->getInterval()));

	return nowTime > nextExecutionTime;
}

void ScriptHost::deleteScript(string scriptName)
{
	lock_recursive(_scriptsMutex);

	PythonScriptPtr script = getScript(scriptName);

	if (!scriptExists(scriptName))
		return;

	_scripts.erase(script);
}

void ScriptHost::addDiagnosticStatus(PythonScriptPtr script)
{
	lock_recursive(_statusesMutex);

	DiagnosticStatusPtr status(new diagnostic_msgs::DiagnosticStatus());

	status->name = script->getName();
	status->hardware_id = script->getParameter("hardware_id");

	if (script->isValidationFailed()) {
		status->level = (int8_t)script->getFailType();
	}
	else
		status->level = diagnostic_msgs::DiagnosticStatus::OK;

	status->message = script->isValidationFailed() ? "Validation failed: " + script->getFailedValidation() : "Fine";;

	_diagnosticStatuses.push_back(status);
}

vector<DiagnosticStatusPtr> ScriptHost::getDiagnosticStatusesAndClear()
{
	lock_recursive(_statusesMutex);

	vector<DiagnosticStatusPtr> statuses = _diagnosticStatuses;
	_diagnosticStatuses.clear();
	return statuses;
}

set<PythonScriptPtr> ScriptHost::getScripts()
{
	lock_recursive(_scriptsMutex);
	return _scripts;
}

PythonScriptPtr ScriptHost::getScript(string scriptName)
{
	lock_recursive(_scriptsMutex);

	for(std::set<PythonScriptPtr>::iterator it = _scripts.begin(); it != _scripts.end(); it++) {
		if ((*it)->getName() == scriptName)
			return (*it);
	}

	return PythonScriptPtr();
}

bool ScriptHost::hasAllTopicValues(PythonScriptPtr script)
{
	foreach(string topicName, script->getUsedTopics()) {
		if (!RosTopicListener::hasTopicValue(topicName))
			return false;
	}

	return true;
}

bool ScriptHost::scriptExists(string scriptName)
{
	lock_recursive(_scriptsMutex);
	return !(!getScript(scriptName));
}
