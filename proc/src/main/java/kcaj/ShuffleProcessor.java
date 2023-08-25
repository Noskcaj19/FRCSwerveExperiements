// package kcaj;

// import java.util.LinkedHashSet;
// import java.util.Map;
// import java.util.Set;

// import com.sun.source.util.JavacTask;
// import com.sun.source.util.TaskEvent;
// import com.sun.source.util.TaskListener;
// import com.sun.source.util.TreePath;
// import com.sun.source.util.Trees;

// import javax.annotation.processing.AbstractProcessor;
// import javax.annotation.processing.Completion;
// import javax.annotation.processing.Filer;
// import javax.annotation.processing.Messager;
// import javax.annotation.processing.ProcessingEnvironment;
// import javax.annotation.processing.Processor;
// import javax.annotation.processing.RoundEnvironment;
// import javax.annotation.processing.SupportedAnnotationTypes;
// import javax.annotation.processing.SupportedSourceVersion;
// import javax.lang.model.SourceVersion;
// import javax.lang.model.element.AnnotationMirror;
// import javax.lang.model.element.Element;
// import javax.lang.model.element.ExecutableElement;
// import javax.lang.model.element.TypeElement;
// import javax.lang.model.util.Elements;
// import javax.lang.model.util.Types;
// import javax.tools.Diagnostic;

// import com.google.auto.service.AutoService;
// import com.google.common.collect.ImmutableSet;

// // @SupportedAnnotationTypes({
// //         "kcaj.Test",
// // })
// // @SupportedSourceVersion(SourceVersion.RELEASE_11)
// @AutoService(Processor.class)
// public class ShuffleProcessor extends AbstractProcessor {
//   // @Override public Set<String> getSupportedAnnotationTypes() {
//   //   Set<String> annotations = new LinkedHashSet<>();
//   //   annotations.add(Test.class.getCanonicalName());
//   //   return annotations;
//   // }


//   private final class TaskListenerImplementation implements TaskListener {
// private ShuffleProcessor processor;

// TaskListenerImplementation(ShuffleProcessor processor) {
//   this.processor = processor;

// }

//     @Override
//     public void finished(TaskEvent e) {
//         if (e.getKind() != TaskEvent.Kind.ANALYZE) {
//           return;
//       }
// final TreePath treePath = processor.trees.getPath(e.getTypeElement());
// System.out.println(e.getTypeElement());
// System.out.println("P"+treePath);
//     }
//   }

//   public Trees trees;

//   @Override
//   public ImmutableSet<String> getSupportedAnnotationTypes() {
//     return ImmutableSet.of(Test.class.getName());
//   }

//   @Override
//   public SourceVersion getSupportedSourceVersion() {
//     return SourceVersion.latestSupported();
//   }

//   @Override
//   public synchronized void init(ProcessingEnvironment processingEnv) {
//       super.init(processingEnv);
//       trees = Trees.instance(processingEnv);

//       JavacTask.instance(processingEnv).addTaskListener(new TaskListenerImplementation(this));
//   }

//   @Override
//   public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
//   if (roundEnv.processingOver()) {
//     System.out.println("Processing over");
//     return false;
//   }
//   for (TypeElement annotation : annotations) {
    
//     Set<? extends Element> annotatedElements = roundEnv.getElementsAnnotatedWith(annotation);
//     // System.out.println(annotation);
//     System.out.println("a"+annotatedElements);
//     System.out.println("r"+roundEnv.getRootElements());
//     System.out.println("t"+roundEnv.getClass());
// }
// processingEnv.getMessager().printMessage(Diagnostic.Kind.WARNING,"Hey");

//   return false;
//   }

//   @Override
//   public Iterable<? extends Completion> getCompletions(Element element, AnnotationMirror annotation,
//           ExecutableElement member, String userText) {
//       // TODO Auto-generated method stub
//       return super.getCompletions(element, annotation, member, userText);
//   }

// }
