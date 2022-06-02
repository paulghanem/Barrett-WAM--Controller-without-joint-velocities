
training_images=imdb_mnist.images.data(:,:,1:50000);
training_labels=imdb_mnist.images.labels(1,1:50000);

model=train(training_images,training_labels);



function Model=train(training, groupnames)
            %SVMTRAIN Train a support vector machine classifier
            %   SVMSTRUCT = SVMTRAIN(TRAINING, Y) trains a support vector machine (SVM)
            %   classifier on data taken from two groups. TRAINING is a numeric matrix
            %   of predictor data. Rows of TRAINING correspond to observations; columns
            %   correspond to features. Y is a column vector that contains the known
            %   class labels for TRAINING. Y is a grouping variable, i.e., it can be a
            %   categorical, numeric, or logical vector; a cell vector of strings; or a
            %   character matrix with each row representing a class label (see help for
            %   groupingvariable). Each element of Y specifies the group the
            %   corresponding row of TRAINING belongs to. TRAINING and Y must have the
            %   same number of rows. SVMSTRUCT contains information about the trained
            %   classifier, including the support vectors, that is used by SVMCLASSIFY
            %   for classification. SVMTRAIN treats NaNs, empty strings or 'undefined'
            %   values as missing values and ignores the corresponding rows in
            %   TRAINING and Y.
            %
            %   SVMSTRUCT = SVMTRAIN(TRAINING, Y, 'PARAM1',val1, 'PARAM2',val2, ...)
            %   specifies one or more of the following name/value pairs:
            %
            %      Name                Value
            %      'kernel_function'  A string or a function handle specifying the
            %                         kernel function used to represent the dot
            %                         product in a new space. The value can be one of
            %                         the following:
            %                         'linear'     - Linear kernel or dot product
            %                                        (default). In this case, SVMTRAIN
            %                                        finds the optimal separating plane
            %                                        in the original space.
            %                         'quadratic'  - Quadratic kernel
            %                         'polynomial' - Polynomial kernel with default
            %                                        order 3. To specify another order,
            %                                        use the 'polyorder' argument.
            %                         'rbf'        - Gaussian Radial Basis Function
            %                                        with default scaling factor 1. To
            %                                        specify another scaling factor,
            %                                        use the 'rbf_sigma' argument.
            %                         'mlp'        - Multilayer Perceptron kernel (MLP)
            %                                        with default weight 1 and default
            %                                        bias -1. To specify another weight
            %                                        or bias, use the 'mlp_params'
            %                                        argument.
            %                         function     - A kernel function specified using
            %                                        @(for example @KFUN), or an
            %                                        anonymous function. A kernel
            %                                        function must be of the form
            %
            %                                        function K = KFUN(U, V)
            %
            %                                        The returned value, K, is a matrix
            %                                        of size M-by-N, where M and N are
            %                                        the number of rows in U and V
            %                                        respectively.
            %
            %   'rbf_sigma'           A positive number specifying the scaling factor
            %                         in the Gaussian radial basis function kernel.
            %                         Default is 1.
            %
            %   'polyorder'           A positive integer specifying the order of the
            %                         polynomial kernel. Default is 3.
            %
            %   'mlp_params'          A vector [P1 P2] specifying the parameters of MLP
            %                         kernel.  The MLP kernel takes the form:
            %                         K = tanh(P1*U*V' + P2),
            %                         where P1 > 0 and P2 < 0. Default is [1,-1].
            %
            %   'method'              A string specifying the method used to find the
            %                         separating hyperplane. Choices are:
            %                         'SMO' - Sequential Minimal Optimization (SMO)
            %                                 method (default). It implements the L1
            %                                 soft-margin SVM classifier.
            %                         'QP'  - Quadratic programming (requires an
            %                                 Optimization Toolbox license). It
            %                                 implements the L2 soft-margin SVM
            %                                 classifier. Method 'QP' doesn't scale
            %                                 well for TRAINING with large number of
            %                                 observations.
            %                         'LS'  - Least-squares method. It implements the
            %                                 L2 soft-margin SVM classifier.
            %
            %   'options'             Options structure created using either STATSET or
            %                         OPTIMSET.
            %                         * When you set 'method' to 'SMO' (default),
            %                           create the options structure using STATSET.
            %                           Applicable options:
            %                           'Display'  Level of display output.  Choices
            %                                    are 'off' (the default), 'iter', and
            %                                    'final'. Value 'iter' reports every
            %                                    500 iterations.
            %                           'MaxIter'  A positive integer specifying the
            %                                    maximum number of iterations allowed.
            %                                    Default is 15000 for method 'SMO'.
            %                         * When you set method to 'QP', create the
            %                           options structure using OPTIMSET. For details
            %                           of applicable options choices, see QUADPROG
            %                           options. SVM uses a convex quadratic program,
            %                           so you can choose the 'interior-point-convex'
            %                           algorithm in QUADPROG.
            %
            %  'tolkkt'              A positive scalar that specifies the tolerance
            %                        with which the Karush-Kuhn-Tucker (KKT) conditions
            %                        are checked for method 'SMO'. Default is
            %                        1.0000e-003.
            %
            %  'kktviolationlevel'   A scalar specifying the fraction of observations
            %                        that are allowed to violate the KKT conditions for
            %                        method 'SMO'. Setting this value to be positive
            %                        helps the algorithm to converge faster if it is
            %                        fluctuating near a good solution. Default is 0.
            %
            %  'kernelcachelimit'    A positive scalar S specifying the size of the
            %                        kernel matrix cache for method 'SMO'. The
            %                        algorithm keeps a matrix with up to S * S
            %                        double-precision numbers in memory. Default is
            %                        5000. When the number of points in TRAINING
            %                        exceeds S, the SMO method slows down. It's
            %                        recommended to set S as large as your system
            %                        permits.
            %
            %  'boxconstraint'       The box constraint C for the soft margin. C can be
            %                        a positive numeric scalar or a vector of positive
            %                        numbers with the number of elements equal to the
            %                        number of rows in TRAINING.
            %                        Default is 1.
            %                        * If C is a scalar, it is automatically rescaled
            %                          by N/(2*N1) for the observations of group one,
            %                          and by N/(2*N2) for the observations of group
            %                          two, where N1 is the number of observations in
            %                          group one, N2 is the number of observations in
            %                          group two. The rescaling is done to take into
            %                          account unbalanced groups, i.e., when N1 and N2
            %                          are different.
            %                        * If C is a vector, then each element of C
            %                          specifies the box constraint for the
            %                          corresponding observation.
            %
            %   'autoscale'          A logical value specifying whether or not to
            %                        shift and scale the data points before training.
            %                        When the value is true, the columns of TRAINING
            %                        are shifted and scaled to have zero mean unit
            %                        variance. Default is true.
            %
            %   'showplot'           A logical value specifying whether or not to show
            %                        a plot. When the value is true, SVMTRAIN creates a
            %                        plot of the grouped data and the separating line
            %                        for the classifier, when using data with 2
            %                        features (columns). Default is false.
            %
            %   SVMSTRUCT is a structure having the following properties:
            %
            %   SupportVectors       Matrix of data points with each row corresponding
            %                        to a support vector.
            %                        Note: when 'autoscale' is false, this field
            %                        contains original support vectors in TRAINING.
            %                        When 'autoscale' is true, this field contains
            %                        shifted and scaled vectors from TRAINING.
            %   Alpha                Vector of Lagrange multipliers for the support
            %                        vectors. The sign is positive for support vectors
            %                        belonging to the first group and negative for
            %                        support vectors belonging to the second group.
            %   Bias                 Intercept of the hyperplane that separates
            %                        the two groups.
            %                        Note: when 'autoscale' is false, this field
            %                        corresponds to the original data points in
            %                        TRAINING. When 'autoscale' is true, this field
            %                        corresponds to shifted and scaled data points.
            %   KernelFunction       The function handle of kernel function used.
            %   KernelFunctionArgs   Cell array containing the additional arguments
            %                        for the kernel function.
            %   GroupNames           A column vector that contains the known
            %                        class labels for TRAINING. Y is a grouping
            %                        variable (see help for groupingvariable).
            %   SupportVectorIndices A column vector indicating the indices of support
            %                        vectors.
            %   ScaleData            This field contains information about auto-scale.
            %                        When 'autoscale' is false, it is empty. When
            %                        'autoscale' is set to true, it is a structure
            %                        containing two fields:
            %                        shift       - A row vector containing the negative
            %                                      of the mean across all observations
            %                                      in TRAINING.
            %                        scaleFactor - A row vector whose value is
            %                                      1./STD(TRAINING).
            %   FigureHandles        A vector of figure handles created by SVMTRAIN
            %                        when 'showplot' argument is TRUE.
            %
            %   Example:
            %       % Load the data and select features for classification
            %       load fisheriris
            %       X = [meas(:,1), meas(:,2)];
            %       % Extract the Setosa class
            %       Y = nominal(ismember(species,'setosa'));
            %       % Randomly partitions observations into a training set and a test
            %       % set using stratified holdout
            %       P = cvpartition(Y,'Holdout',0.20);
            %       % Use a linear support vector machine classifier
            %       svmStruct = msvmtrain(X(P.training,:),Y(P.training),'showplot',true);
            %       C = msvmclassify(svmStruct,X(P.test,:),'showplot',true);
            %       errRate = sum(Y(P.test)~= C)/P.TestSize  %mis-classification rate
            %       conMat = confusionmat(Y(P.test),C) % the confusion matrix
            %
            %   See also SVMCLASSIFY, NAIVEBAYES, CLASSREGTREE, CLASSIFY, TREEBAGGER,
            %            GROUPINGVARIABLE
            
            %   Copyright 2004-2012 The MathWorks, Inc.
            
            
            %   References:
            %
            %     [1] Cristianini, N., Shawe-Taylor, J An Introduction to Support
            %         Vector Machines, Cambridge University Press, Cambridge, UK. 2000.
            %         http://www.support-vector.net
            %     [2] Kecman, V, Learning and Soft Computing,
            %         MIT Press, Cambridge, MA. 2001.
            %     [3] Suykens, J.A.K., Van Gestel, T., De Brabanter, J., De Moor, B.,
            %         Vandewalle, J., Least Squares Support Vector Machines,
            %         World Scientific, Singapore, 2002.
            %     [4] J.C. Platt: A Fast Algorithm for Training  Support Vector
            %         Machines,  Advances in Kernel Methods - Support Vector Learning,
            %         MIT Press, 1998.
            %     [5] J.C. Platt: Fast Training of Support Vector Machines using
            %         Sequential Minimal Optimization Microsoft Research Technical
            %         Report MSR-TR-98-14, 1998.
            %     [6] http://www.kernel-machines.org/papers/tr-30-1998.ps.gz
            %
            %   SVMTRAIN(...,'KFUNARGS',ARGS) allows you to pass additional
            %   arguments to kernel functions.
            %
            %     Code is modified for multuclass svm
            %         by Er.Abbas Manthiri BE
            %     Email abbasmanthiribe@gmail.com
            %     Date:15-03-2017
            
            classInstance=unique(groupnames);
            svmValue=sum(classInstance);
            nsample=length(classInstance);
            
                model=cell(1,nsample);
                for i=1:nsample
                    classx=groupnames;
                    classx(classx==classInstance(i))=svmValue;
                    classx(classx~=svmValue)=1;
                    classx(classx==svmValue)=0;
                    model{i}=svm(training,classx,'kernel_function','linear');
                    fprintf('Multi Class SVM Model for Class Instance %d --->\n',classInstance(i))
                    disp(model{i})
                end
            
            Model.model=model;
            Model.classInstance=classInstance;
            fprintf('\nTrain Model Completed\n')
            
        end
        
       